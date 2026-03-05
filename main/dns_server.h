#pragma once

/**
 * Start a DNS server that responds to all A-record queries with 192.168.4.1.
 * Used for captive portal detection in AP mode.
 */
void dns_server_start(void);
