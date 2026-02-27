class VoltageReader{

    private :
        volatile bool dataReceived = false;
        volatile char receivedData[32]; // Buffer to hold received data
        volatile int receivedLength = 0;
        
        const int sensorPin_Lipo = A2; // select the input pin for the battery sensor
        const int sensorPin_NiMh = A3; // select the input pin for the battery sensor
        
        const float r1_LiPo = 560;  // resistance of the first resistor
        const float r1_NiMh = 560;  // resistance of the second resistor
        const float r2_LiPo = 1500; // resistance of the second resistor
        const float r2_NiMh = 1000; // resistance of the second resistor
        float voltage_LiPo = 0;     // variable to store the value read
        float voltage_NiMh = 0;     // variable to store the value read
    
    public :
        static void setup();
        static void calculateVoltage();
        static void request();


};