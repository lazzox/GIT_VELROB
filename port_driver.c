#include "Headers/port_driver.h"



/*  Configures interrupt 0.
 *
 *  This function configures interrupt 0 to be associated with a set of pins and
 *  sets the desired interrupt level.
 *
 *  \param port       The port to configure.
 *  \param intLevel   The desired interrupt level for port interrupt 0.
 *  \param pinMask    A mask that selects the pins to associate with port interrupt 0.
 */
void PORT_ConfigureInterrupt0( PORT_t * port,
                               PORT_INT0LVL_t intLevel,
                               uint8_t pinMask )
{
	port->INTCTRL = ( port->INTCTRL & ~PORT_INT0LVL_gm ) | intLevel;
	port->INT0MASK = pinMask;
}


/*  Configures interrupt 1.
 *
 *  This function configures interrupt 1 to be associated with a set of pins and
 *  sets the desired interrupt level.
 *
 *  \param port       The port to configure.
 *  \param intLevel   The desired interrupt level for port interrupt 1.
 *  \param pinMask    A mask that selects the pins to associate with port interrupt 1.
 */
void PORT_ConfigureInterrupt1( PORT_t * port,
                               PORT_INT1LVL_t intLevel,
                               uint8_t pinMask )
{
	port->INTCTRL = ( port->INTCTRL & ~PORT_INT1LVL_gm ) | intLevel;
	port->INT1MASK = pinMask;
}



/*  This function changes the configuration of a set of pins.
 *
 *  \param port             The port.
 *  \param pinMask          A bit mask that selects the pins to configure.
 *  \param slewRateEnable   Enable/disable slew rate control.
 *  \param invertEnable     Enable/disable I/O inversion.
 *  \param opc              Output/Pull Configuration.
 *  \param isc              Input/Sense Configuration.
 */
void PORT_ConfigurePins( PORT_t * port,
                         uint8_t pinMask,
                         uint8_t slewRateEnable,
                         uint8_t invertEnable,
                         PORT_OPC_t opc,
                         PORT_ISC_t isc)
{
	// Build pin control register value
	// N.B. Look in iox126a1.h for description of bit and group masks
	uint8_t temp = (uint8_t) opc |
	               isc |
	               (slewRateEnable ? PORT_SRLEN_bm : 0) |
	               (invertEnable ? PORT_INVEN_bm : 0);

	// Configure the pins in one atomic operation. 

	// Save status register. 
	uint8_t sreg = SREG;

	// Disable the global interrupt flag
	cli();	
	PORTCFG.MPCMASK = pinMask;
	port->PIN0CTRL = temp;

	// Restore status register. 
	SREG = sreg;
}


/*  Maps a real port to virtual port 0.
 *
 *  This function maps a real port to virtual port 0 to make some of the
 *  registers available in I/O space.
 *
 *  \param realPort Selects the real port to map to virtual port 0.
 */
void PORT_MapVirtualPort0(PORTCFG_VP0MAP_t realPort)
{
	PORTCFG.VPCTRLA = ( PORTCFG.VPCTRLA & ~PORTCFG_VP0MAP_gm ) | realPort;
}


/*  Maps a real port to virtual port 1.
 *
 *  This function maps a real port to virtual port 1 to make some of the
 *  registers available in I/O space.
 *
 *  \param realPort Selects the real port to map to virtual port 1.
 */
void PORT_MapVirtualPort1(PORTCFG_VP1MAP_t realPort)
{
	PORTCFG.VPCTRLA = ( PORTCFG.VPCTRLA & ~PORTCFG_VP1MAP_gm ) | realPort;
}


/*  Maps a real port to virtual port 2.
 *
 *  This function maps a real port to virtual port 2 to make some of the
 *  registers available in I/O space.
 *
 *  \param realPort Selects the real port to map to virtual port 2.
 */
void PORT_MapVirtualPort2(PORTCFG_VP2MAP_t realPort)
{
	PORTCFG.VPCTRLB = ( PORTCFG.VPCTRLB & ~PORTCFG_VP2MAP_gm ) | realPort;
}


/*  Maps a real port to virtual port 3.
 *
 *  This function maps a real port to virtual port 3 to make some of the
 *  registers available in I/O space.
 *
 *  \param realPort Selects the real port to map to virtual port 3.
 */
void PORT_MapVirtualPort3(PORTCFG_VP3MAP_t realPort)
{
	PORTCFG.VPCTRLB = ( PORTCFG.VPCTRLB & ~PORTCFG_VP3MAP_gm ) | realPort;
}
