#ifndef SCANNER_H_
#define SCANNER_H_

#include <string>
#include "GNSocket.h"

class Scanner
{
public:
    void connect(const char *ip, unsigned port);
    bool barcode(std::string &code);

private:
   GNSocket m_socket;
};

#endif //SCANNER_H_


