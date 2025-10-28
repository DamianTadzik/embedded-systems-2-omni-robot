# LL README 

Hi this is reallll... fuuunny 

Jettson is talking to the arduino over UART, classical 9800 8n1

The communication protocol frame consists of:
- Control [1] bytes
- payload [4 - 9] bytes

NONE OF PAYLOAD BYTES SHOULD CARRY VALUE GREATER EQUAL THAN 240

# Available control modes:

- 240
    - Requires 8 bytes of payload
    - ?? i am really not into debugging that topic

- 245
    - Requires 5 bytes of payload
    - First four bytes contain encoded wheel speed 
        - Range of decoded signal is [-100, 100]
        - Range of encoded signal is just [0, 200]
        - Encoding decoding is just done by subtraction/addition
    - Last byte contains encoded multiplier, 
        - Encoder resolution is 240 counts per revolution
        - 
    - Control signal for each motor consists of the direction bit and one byte (0-255 range) duty cycle
    



- 250
