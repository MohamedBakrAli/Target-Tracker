#ifndef COMMUNICATION_H
#define COMMUNICATION_H


class Communication {


public:
  Communication ();
  void transmit_to_slave (float,const char*);
  int  receive_data (void);
};


#endif
