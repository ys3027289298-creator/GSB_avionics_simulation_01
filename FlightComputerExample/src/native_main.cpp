#include <stdio.h>
#include "Hal.h"

void setup();
void loop();

int main() {
    setup();

    while (true) {
        loop();
    }

    return 0;
}