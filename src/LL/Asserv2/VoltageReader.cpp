class VoltageReader{
    void setup(){
        pinMode(sensorPin_Lipo, INPUT);
        pinMode(sensorPin_NiMh, INPUT);
    }
    void calculateVoltage(){
        //read from the sensor
        // and convert the value to voltage
        voltage_LiPo = analogRead(sensorPin_Lipo);
        voltage_NiMh = analogRead(sensorPin_NiMh);
        voltage_LiPo = voltage_LiPo * (5.0 / 1023.0) * ((r1_LiPo + r2_LiPo) / r1_LiPo);
        voltage_NiMh = voltage_NiMh * (5.0 / 1023.0) * ((r1_NiMh + r2_NiMh) / r1_NiMh);
    }
    void request(){
        calculateVoltage();
        const int numFloats = 2; // Number of floats to send
        float data[numFloats] = {voltage_LiPo, voltage_NiMh}; // Example float values to send
        byte* dataBytes = (byte*)data;
    }
    
}