std_msgs/Header header

uint32 instance_id

float32 system_cpu # total cpu utilization

float32 core_temperature # cpu temperature

uint32 system_ram_total # total ram

uint32 system_ram_used # used ram

uint32 system_ram_free # free ram

float32 system_io_in # disk input speed

float32 system_io_out # disk output speed

string[] system_ipv4_ip # ipv4 address

float32 system_ipv4_received # ipv4 received bandwidth

float32 system_ipv4_sent # ipv4 sent bandwidth

uint16 ipv4_tcpsock # number of tcp active connections

float32 ipv4_tcppackets_received # number of tcp received packages

float32 ipv4_tcppackets_sent # number of tcp sent packages

float32 ipv4_tcp_errors # number of tcp error packages

float32 ipv4_udppackets_received # number of udp received packages

float32 ipv4_udppackets_sent # number of udp sent packages

float32 ipv4_udp_errors # number of udp error packages

float32 cpu_interrupts

float32 cpu_context_switches

uint32 softnet_processed

uint32 softnet_dropped

uint32 softnet_squeezed

uint32 softnet_received_rps

uint32 softnet_flow_limit_count

uint64 softirqs_hi

uint64 softirqs_timer

uint64 softirqs_net_tx

uint64 softirqs_net_rx

uint64 softirqs_block

uint64 softirqs_irq_poll

uint64 softirqs_tasklet

uint64 softirqs_sched

uint64 softirqs_hrtimer

uint64 softirqs_rcu

float32 load1

float32 load5

float32 load15

