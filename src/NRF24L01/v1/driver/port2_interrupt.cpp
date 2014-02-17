#include <msp430.h>

void (*port2_interrupt_function)(void*) = 0;
void* port2_interrupt_payload = 0;

#pragma vector=PORT2_VECTOR
__interrupt void PORT2_PCI(void)
{
  if (port2_interrupt_function)
  {
    port2_interrupt_function(port2_interrupt_payload);
  }

  // Clear LPM4 power save mode bit
  __bic_SR_register_on_exit(LPM4_bits);
}
