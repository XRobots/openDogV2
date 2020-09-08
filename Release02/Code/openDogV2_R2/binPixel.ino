void binPixel(int dec) {
    // convert dec to binary and display on pixels

    int bit0 = bitRead(dec, 0);
    int bit1 = bitRead(dec, 1);
    int bit2 = bitRead(dec, 2);
    int bit3 = bitRead(dec, 3);

    if (bit0 == 0) {
      leds.setPixel(0, BLACK);
    }
    else {
      leds.setPixel(0, RED);
    }

    //***********

    if (bit1 == 0) {
      leds.setPixel(1, BLACK);
    }
    else {
      leds.setPixel(1, RED);
    }

    //***********

    if (bit2 == 0) {
      leds.setPixel(2, BLACK);
    }
    else {
      leds.setPixel(2, RED);
    }

    //***********

    if (bit3 == 0) {
      leds.setPixel(3, BLACK);
    }
    else {
      leds.setPixel(3, RED);
    }
    
    leds.show();  
}

void modeSelect(int dec) {

    int bit0 = bitRead(dec, 0);
    int bit1 = bitRead(dec, 1);
    int bit2 = bitRead(dec, 2);
    int bit3 = bitRead(dec, 3);

    if (bit0 == 0) {
      leds.setPixel(7, BLACK);
    }
    else {
      leds.setPixel(7, BLUE);
    }

    //***********

    if (bit1 == 0) {
      leds.setPixel(6, BLACK);
    }
    else {
      leds.setPixel(6, BLUE);
    }

    //***********

    if (bit2 == 0) {
      leds.setPixel(5, BLACK);
    }
    else {
      leds.setPixel(5, BLUE);
    }

    //***********

    if (bit3 == 0) {
      leds.setPixel(4, BLACK);
    }
    else {
      leds.setPixel(4, BLUE);
    }
    
    leds.show(); 
    
}

    

