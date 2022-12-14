/******************************************************************************
* File Name:   udp_client.h
*
* Description: This file contains declaration of task related to UDP client
*              operation.
*
********************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef UDP_CLIENT_H_
#define UDP_CLIENT_H_

/*******************************************************************************
* Macros
********************************************************************************/

#define MAKE_IPV4_ADDRESS(a, b, c, d)     ((((uint32_t) d) << 24) | \
                                          (((uint32_t) c) << 16) | \
                                          (((uint32_t) b) << 8) |\
                                          ((uint32_t) a))

/* Set the destination IP address and destination port number
 * to that of the UDP server
 */
#define UDP_SERVER_IP_ADDRESS             MAKE_IPV4_ADDRESS(192, 168, 1, 11)
#define UDP_SERVER_PORT                   (50007)

/* Set to 1 to use static IP addressing for UDP client */
#define ENABLE_STATIC_IP_ADDRESS (0u)

#if ENABLE_STATIC_IP_ADDRESS
/* Set the static IP address of the UDP client keeping the gateway
 * and subnet mask address same as that of UDP server
 */
#define UDP_STATIC_IP_ADDR   MAKE_IPV4_ADDRESS( 192, 168, 1, 8 )
#define UDP_STATIC_GATEWAY   MAKE_IPV4_ADDRESS( 192, 168, 1, 1 )
#define UDP_NETMASK          MAKE_IPV4_ADDRESS( 255, 255, 255, 0 )
#endif

/*******************************************************************************
* Function Prototype
********************************************************************************/
void udp_client_task(void *arg);

#endif /* UDP_CLIENT_H_ */

/* [] END OF FILE */
