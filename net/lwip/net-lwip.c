// SPDX-License-Identifier: GPL-2.0

/* Copyright (C) 2024 Linaro Ltd. */

#include <command.h>
#include <env.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <hexdump.h>
#include <linux/compiler_attributes.h>
#include <linux/kernel.h>
#include <lwip/ip4_addr.h>
#include <lwip/dns.h>
#include <lwip/err.h>
#include <lwip/netif.h>
#include <lwip/pbuf.h>
#include <lwip/etharp.h>
#include <lwip/init.h>
#include <lwip/prot/ip.h>
#include <lwip/prot/ip4.h>
#include <lwip/prot/iana.h>
#include <lwip/prot/udp.h>
#include <lwip/prot/etharp.h>
#include <lwip/timeouts.h>
#include <net.h>
#include <timer.h>
#include <u-boot/schedule.h>

/* xx:xx:xx:xx:xx:xx\0 */
#define MAC_ADDR_STRLEN 18

#if defined(CONFIG_API) || defined(CONFIG_EFI_LOADER)
void (*push_packet)(void *, int len) = 0;
#endif
int net_try_count;
static int net_restarted;
int net_restart_wrap;
static int net_lwip_eth_started;
static uchar net_pkt_buf[(PKTBUFSRX) * PKTSIZE_ALIGN + PKTALIGN]
	__aligned(PKTALIGN);
/* NO_SYS lwIP serializes linkoutput calls and U-Boot drivers copy on send. */
static uchar net_tx_pkt_buf[PKTSIZE_ALIGN] __aligned(PKTALIGN);
const u8 net_bcast_ethaddr[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
char *pxelinux_configfile;

#if defined(CONFIG_HTTPD_RECOVERY)
static net_lwip_udp_recv_fn recovery_dhcp_hook;
static void *recovery_dhcp_hook_arg;

void net_lwip_set_recovery_dhcp_hook(net_lwip_udp_recv_fn recv, void *arg)
{
	recovery_dhcp_hook = recv;
	recovery_dhcp_hook_arg = arg;
}
#else
void net_lwip_set_recovery_dhcp_hook(net_lwip_udp_recv_fn recv, void *arg)
{
}
#endif

#if defined(CONFIG_HTTPD_RECOVERY)
static bool net_lwip_dispatch_recovery_dhcp(const uchar *packet, int len)
{
	const struct eth_hdr *eth;
	const struct ip_hdr *ip;
	const struct udp_hdr *udp;
	const uchar *payload;
	struct pbuf *pbuf;
	ip_addr_t src_addr;
	ip4_addr_t src_ip;
	int ip_hlen, udp_len, payload_len, offset;

	if (!recovery_dhcp_hook ||
	    len < (int)(SIZEOF_ETH_HDR + sizeof(struct ip_hdr) +
			sizeof(struct udp_hdr)))
		return false;

	eth = (const struct eth_hdr *)packet;
	if (lwip_ntohs(eth->type) != ETHTYPE_IP)
		return false;

	ip = (const struct ip_hdr *)(packet + SIZEOF_ETH_HDR);
	if (IPH_PROTO(ip) != IP_PROTO_UDP)
		return false;

	ip_hlen = IPH_HL_BYTES(ip);
	offset = SIZEOF_ETH_HDR + ip_hlen;
	if (len < offset + (int)sizeof(*udp))
		return false;

	udp = (const struct udp_hdr *)(packet + offset);
	if (lwip_ntohs(udp->src) != LWIP_IANA_PORT_DHCP_CLIENT ||
	    lwip_ntohs(udp->dest) != LWIP_IANA_PORT_DHCP_SERVER)
		return false;

	udp_len = lwip_ntohs(udp->len);
	if (udp_len < (int)sizeof(*udp))
		return false;

	payload_len = udp_len - sizeof(*udp);
	if (len < offset + udp_len)
		payload_len = len - offset - sizeof(*udp);
	if (payload_len <= 0)
		return false;

	pbuf = pbuf_alloc(PBUF_TRANSPORT, payload_len, PBUF_RAM);
	if (!pbuf)
		return false;

	payload = packet + offset + sizeof(*udp);
	if (pbuf_take(pbuf, payload, payload_len) != ERR_OK) {
		pbuf_free(pbuf);
		return false;
	}

	IPADDR_WORDALIGNED_COPY_TO_IP4_ADDR_T(&src_ip, &ip->src);
	ip_addr_copy_from_ip4(src_addr, src_ip);
	recovery_dhcp_hook(recovery_dhcp_hook_arg, NULL, pbuf, &src_addr,
			   lwip_ntohs(udp->src));
	return true;
}
#else
static bool net_lwip_dispatch_recovery_dhcp(const uchar *packet, int len)
{
	return false;
}
#endif

static err_t net_lwip_tx(struct netif *netif, struct pbuf *p)
{
	struct udevice *udev = netif->state;
	void *packet = p->payload;
	u16_t tx_len = p->tot_len;
	int err;

	if (CONFIG_IS_ENABLED(LWIP_DEBUG_RXTX)) {
		printf("net_lwip_tx: %u bytes, udev %s\n", tx_len, udev->name);
		print_hex_dump("net_lwip_tx: ", 0, 16, 1, p->payload, p->len,
			       true);
	}

	/*
	 * lwIP may hand us a chained pbuf for TCP payload. The U-Boot ethernet
	 * drivers expect one contiguous frame buffer, so flatten the full
	 * packet whenever the first payload is unaligned or the pbuf is chained.
	 */
	if (p->next || ((unsigned long)p->payload % PKTALIGN)) {
		if (tx_len > sizeof(net_tx_pkt_buf))
			return ERR_BUF;
		if (pbuf_copy_partial(p, net_tx_pkt_buf, tx_len, 0) != tx_len)
			return ERR_ABRT;
		packet = net_tx_pkt_buf;
	}

	err = eth_get_ops(udev)->send(udev, packet, tx_len);
	if (err) {
		debug("send error %d\n", err);
		return ERR_ABRT;
	}

	return ERR_OK;
}

static err_t net_lwip_if_init(struct netif *netif)
{
	netif->output = etharp_output;
	netif->linkoutput = net_lwip_tx;
	netif->mtu = 1500;
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

	return ERR_OK;
}

static void eth_init_rings(void)
{
	int i;

	for (i = 0; i < PKTBUFSRX; i++)
		net_rx_packets[i] = net_pkt_buf + i  * PKTSIZE_ALIGN;
}

struct netif *net_lwip_get_netif(void)
{
	struct netif *netif, *found = NULL;

	NETIF_FOREACH(netif) {
		if (!found)
			found = netif;
		else
			printf("Error: more than one netif in lwIP\n");
	}
	return found;
}

static int get_udev_ipv4_info(struct udevice *dev, ip4_addr_t *ip,
			      ip4_addr_t *mask, ip4_addr_t *gw)
{
	char ipstr[] = "ipaddr\0\0\0";
	char maskstr[] = "netmask\0\0\0";
	char gwstr[] = "gatewayip\0\0\0";
	int idx = dev_seq(dev);
	char *env;

	if (idx < 0 || idx > 99) {
		log_err("unexpected idx %d\n", idx);
		return -1;
	}

	if (idx) {
		sprintf(ipstr, "ipaddr%d", idx);
		sprintf(maskstr, "netmask%d", idx);
		sprintf(gwstr, "gatewayip%d", idx);
	}

	ip4_addr_set_zero(ip);
	ip4_addr_set_zero(mask);
	ip4_addr_set_zero(gw);

	env = env_get(ipstr);
	if (env)
		ip4addr_aton(env, ip);

	env = env_get(maskstr);
	if (env)
		ip4addr_aton(env, mask);

	env = env_get(gwstr);
	if (env)
		ip4addr_aton(env, gw);

	return 0;
}

/*
 * Initialize DNS via env
 */
int net_lwip_dns_init(void)
{
#if CONFIG_IS_ENABLED(DNS)
	bool has_server = false;
	ip_addr_t ns;
	char *nsenv;

	nsenv = env_get("dnsip");
	if (nsenv && ipaddr_aton(nsenv, &ns)) {
		dns_setserver(0, &ns);
		has_server = true;
	}

	nsenv = env_get("dnsip2");
	if (nsenv && ipaddr_aton(nsenv, &ns)) {
		dns_setserver(1, &ns);
		has_server = true;
	}

	if (!has_server) {
		log_err("No valid name server (dnsip/dnsip2)\n");
		return -EINVAL;
	}

	return 0;
#else
	log_err("DNS disabled\n");
	return -EINVAL;
#endif
}

/*
 * Initialize the network stack if needed and start the current device if valid
 */
int net_lwip_eth_start(void)
{
	int ret;

	if (net_lwip_eth_started++ > 0)
		return 0;

	net_init();
	eth_halt();
	eth_set_current();
	ret = eth_init();
	if (ret < 0) {
		net_lwip_eth_started--;
		eth_halt();
		return ret;
	}

	return 0;
}

void net_lwip_eth_stop(void)
{
	if (!net_lwip_eth_started)
		return;

	if (--net_lwip_eth_started)
		return;

	eth_halt();
}

static struct netif *new_netif(struct udevice *udev, bool with_ip)
{
	struct eth_pdata *pdata;
	unsigned char enetaddr[ARP_HLEN] = { 0 };
	char hwstr[MAC_ADDR_STRLEN];
	ip4_addr_t ip, mask, gw;
	struct netif *netif;
	int ret = 0;

	if (!udev)
		return NULL;

	if (eth_start_udev(udev) < 0) {
		log_err("Could not start %s\n", udev->name);
		return NULL;
	}

	netif_remove(net_lwip_get_netif());

	ip4_addr_set_zero(&ip);
	ip4_addr_set_zero(&mask);
	ip4_addr_set_zero(&gw);

	if (with_ip)
		if (get_udev_ipv4_info(udev, &ip, &mask, &gw) < 0)
			return NULL;

	pdata = dev_get_plat(udev);
	if (!eth_env_get_enetaddr_by_index("eth", dev_seq(udev), enetaddr) &&
	    pdata && is_valid_ethaddr(pdata->enetaddr))
		memcpy(enetaddr, pdata->enetaddr, ARP_HLEN);

	if (!is_valid_ethaddr(enetaddr)) {
		printf("error: no valid MAC address for %s\n", udev->name);
		return NULL;
	}

	ret = snprintf(hwstr, MAC_ADDR_STRLEN, "%pM",  enetaddr);
	if (ret < 0 || ret >= MAC_ADDR_STRLEN)
		return NULL;

	netif = calloc(1, sizeof(struct netif));
	if (!netif)
		return NULL;

	netif->name[0] = 'e';
	netif->name[1] = 't';

	string_to_enetaddr(hwstr, netif->hwaddr);
	netif->hwaddr_len = ETHARP_HWADDR_LEN;
	debug("adding lwIP netif for %s with hwaddr:%s ip:%s ", udev->name,
	      hwstr, ip4addr_ntoa(&ip));
	debug("mask:%s ", ip4addr_ntoa(&mask));
	debug("gw:%s\n", ip4addr_ntoa(&gw));

	if (!netif_add(netif, &ip, &mask, &gw, udev, net_lwip_if_init,
		       netif_input)) {
		printf("error: netif_add() failed\n");
		free(netif);
		return NULL;
	}

	netif_set_up(netif);
	netif_set_link_up(netif);
	/* Routing: use this interface to reach the default gateway */
	netif_set_default(netif);

	return netif;
}

struct netif *net_lwip_new_netif(struct udevice *udev)
{
	return new_netif(udev, true);
}

struct netif *net_lwip_new_netif_noip(struct udevice *udev)
{
	return new_netif(udev, false);
}

void net_lwip_remove_netif(struct netif *netif)
{
	netif_remove(netif);
	free(netif);
}

/*
 * Initialize the network buffers, an ethernet device, and the lwIP stack
 * (once).
 */
int net_init(void)
{
	static bool init_done;

	if (!init_done) {
		eth_init_rings();
		lwip_init();
		init_done = true;
	}

	return 0;
}

static struct pbuf *alloc_pbuf_and_copy(uchar *data, int len)
{
	struct pbuf *p, *q;

	/* We allocate a pbuf chain of pbufs from the pool. */
	p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
	if (!p) {
		debug("Failed to allocate pbuf !!!!!\n");
		LINK_STATS_INC(link.memerr);
		LINK_STATS_INC(link.drop);
		return NULL;
	}

	for (q = p; q != NULL; q = q->next) {
		memcpy(q->payload, data, q->len);
		data += q->len;
	}

	LINK_STATS_INC(link.recv);

	return p;
}

int net_lwip_rx(struct udevice *udev, struct netif *netif)
{
	struct pbuf *pbuf;
	uchar *packet;
	int flags;
	int len;
	int i;

	/* lwIP timers */
	sys_check_timeouts();
	/* Other tasks and actions */
	schedule();

	if (!eth_is_active(udev))
		return -EINVAL;

	flags = ETH_RECV_CHECK_DEVICE;
	for (i = 0; i < ETH_PACKETS_BATCH_RECV; i++) {
		net_lwip_run_recovery_poll_hook();
		len = eth_get_ops(udev)->recv(udev, flags, &packet);
		flags = 0;

		if (len > 0) {
			if (net_lwip_dispatch_recovery_dhcp(packet, len)) {
				if (eth_get_ops(udev)->free_pkt)
					eth_get_ops(udev)->free_pkt(udev, packet, len);
				continue;
			}

			if (CONFIG_IS_ENABLED(LWIP_DEBUG_RXTX)) {
				printf("net_lwip_tx: %u bytes, udev %s \n", len,
				       udev->name);
				print_hex_dump("net_lwip_rx: ", 0, 16, 1,
					       packet, len, true);
			}

			pbuf = alloc_pbuf_and_copy(packet, len);
			if (pbuf)
				netif->input(pbuf, netif);
		}
		if (len >= 0 && eth_get_ops(udev)->free_pkt)
			eth_get_ops(udev)->free_pkt(udev, packet, len);
		if (len <= 0)
			break;
	}
	if (len == -EAGAIN)
		len = 0;

	return len;
}

/**
 * net_lwip_dns_resolve() - find IP address from name or IP
 *
 * @name_or_ip: host name or IP address
 * @ip: output IP address
 *
 * Return value: 0 on success, -1 on failure.
 */
int net_lwip_dns_resolve(char *name_or_ip, ip_addr_t *ip)
{
#if defined(CONFIG_DNS)
	char *var = "_dnsres";
	char *argv[] = { "dns", name_or_ip, var, NULL };
	int argc = ARRAY_SIZE(argv) - 1;
#endif

	if (ipaddr_aton(name_or_ip, ip))
		return 0;

#if defined(CONFIG_DNS)
	if (do_dns(NULL, 0, argc, argv) != CMD_RET_SUCCESS)
		return -1;

	name_or_ip = env_get(var);
	if (!name_or_ip)
		return -1;

	if (!ipaddr_aton(name_or_ip, ip))
		return -1;

	env_set(var, NULL);

	return 0;
#else
	return -1;
#endif
}

void net_process_received_packet(uchar *in_packet, int len)
{
#if defined(CONFIG_API) || defined(CONFIG_EFI_LOADER)
	if (push_packet)
		(*push_packet)(in_packet, len);
#endif
}

int net_loop(enum proto_t protocol)
{
	char *argv[1];

	switch (protocol) {
	case TFTPGET:
		argv[0] = "tftpboot";
		return do_tftpb(NULL, 0, 1, argv);
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

u32_t sys_now(void)
{
#if CONFIG_IS_ENABLED(SANDBOX_TIMER)
	return timer_early_get_count();
#else
	return get_timer(0);
#endif
}

int net_start_again(void)
{
	char *nretry;
	int retry_forever = 0;
	unsigned long retrycnt = 0;

	nretry = env_get("netretry");
	if (nretry) {
		if (!strcmp(nretry, "yes"))
			retry_forever = 1;
		else if (!strcmp(nretry, "no"))
			retrycnt = 0;
		else if (!strcmp(nretry, "once"))
			retrycnt = 1;
		else
			retrycnt = simple_strtoul(nretry, NULL, 0);
	} else {
		retrycnt = 0;
		retry_forever = 0;
	}

	if ((!retry_forever) && (net_try_count > retrycnt)) {
		eth_halt();
		/*
		 * We don't provide a way for the protocol to return an error,
		 * but this is almost always the reason.
		 */
		return -ETIMEDOUT;
	}

	net_try_count++;

	eth_halt();
#if !defined(CONFIG_NET_DO_NOT_TRY_ANOTHER)
	eth_try_another(!net_restarted);
#endif
	return eth_init();
}
