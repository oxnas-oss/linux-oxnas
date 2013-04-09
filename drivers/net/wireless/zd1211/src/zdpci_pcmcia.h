#ifndef _ZDPCI_PCMCIA_H
#define _ZDPCI_PCMCIA_H


#ifndef DRIVER_NAME
#define DRIVER_NAME             "zd1205_cb"
#endif


dev_node_t *zdpci_attach( dev_locator_t * );
void zdpci_suspend( dev_node_t * );
void zdpci_resume( dev_node_t * );
void zdpci_detach( dev_node_t * );
int init_module( void );
void cleanup_module( void );


#endif  // _ZDPCI_PCMCIA_H
