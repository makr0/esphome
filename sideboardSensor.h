#include "esphome.h"

class SideboardSensor : public Component, public UARTDevice {
 public:
    SideboardSensor(UARTComponent *parent) : UARTDevice(parent) {}

    static const uint16_t START_FRAME = 0xABCD;  // [-] Start frame definition for reliable serial communication
    uint8_t idx = 0;                            // Index for new data pointer
    uint16_t bufStartFrame;                     // Buffer Start Frame
    char *p;                                    // Pointer declaration for the new received data
    char incomingByte;
    char incomingBytePrev;
    char counter = 0;
    struct SerialFeedback {
        uint16_t start;
        int16_t  cmd1;
        int16_t  cmd2;
        int16_t  speedR_meas;
        int16_t  speedL_meas;
        int16_t  batVoltage;
        int16_t  boardTemp;
        uint16_t cmdLed;
        uint16_t checksum;
    };
    SerialFeedback ReceivedStructure;

    Sensor *cmd1 = new Sensor();
    Sensor *cmd2 = new Sensor();
    Sensor *speed = new Sensor();
    // Sensor *speedR = new Sensor();
    Sensor *batVoltage = new Sensor();
    Sensor *boardTemp = new Sensor();
    Sensor *cmdLed = new Sensor();
    
    void loop() override {
        // Check for new data availability in the Serial buffer
        if (available()) {
            incomingByte 	= read();                                   // Read the incoming byte
            bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
        }
        else {
            return;
        }

        // Copy received data
        if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
            p       = (char *)&ReceivedStructure;
            *p++    = incomingBytePrev;
            *p++    = incomingByte;
            idx     = 2;	
        } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
            *p++    = incomingByte; 
            idx++;
        }	
        
        // Check if we reached the end of the package
        if (idx == sizeof(SerialFeedback)) {
            uint16_t checksum;
            checksum = (uint16_t)(ReceivedStructure.start ^ ReceivedStructure.cmd1 ^ ReceivedStructure.cmd2 ^ ReceivedStructure.speedR_meas ^ ReceivedStructure.speedL_meas
                                ^ ReceivedStructure.batVoltage ^ ReceivedStructure.boardTemp ^ ReceivedStructure.cmdLed);

            // Check validity of the new data
            if (ReceivedStructure.start == START_FRAME && checksum == ReceivedStructure.checksum && counter++==2) {
                // publish data
                // do not send all data, this takes too long
//                cmd1->publish_state(ReceivedStructure.cmd1);
//                cmd2->publish_state(ReceivedStructure.cmd2);
                speed->publish_state((float)(ReceivedStructure.speedR_meas + ReceivedStructure.speedL_meas)/2.0);
                batVoltage->publish_state((float)ReceivedStructure.batVoltage/100);
//                boardTemp->publish_state((float)ReceivedStructure.boardTemp/10);
                cmdLed->publish_state(ReceivedStructure.cmdLed);
                counter = 0;
            }
            idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
        }

        // Update previous states
        incomingBytePrev = incomingByte;
    }
};

