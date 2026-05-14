const int xInput = 33;   // GPIO32 (ADC4)
const int yInput = 32;   // GPIO33 (ADC5)
const int zInput = 34;   // GPIO34 (ADC6)

// ESP32 ADC is typically 12-bit: 0 to 4095
int RawMin = 0;
int RawMax = 4095;

// Take multiple samples to reduce noise
const int sampleSize = 10;

void setup()
{
    Serial.begin(9600);

    // For reading analog
    analogReadResolution(12);

    // Set attenuation so the ADC can read a wider voltage range
    analogSetPinAttenuation(xInput, ADC_11db);
    analogSetPinAttenuation(yInput, ADC_11db);
    analogSetPinAttenuation(zInput, ADC_11db);
}

void loop()
{
    // Read raw values
    int xRaw = ReadAxis(xInput);
    int yRaw = ReadAxis(yInput);
    int zRaw = ReadAxis(zInput);

    // Convert raw values to milli-Gs
    long xScaled = map(xRaw, RawMin, RawMax, -3000, 3000); // +-3g x-axis
    long yScaled = map(yRaw, RawMin, RawMax, -3000, 3000); // +-3g y-axis
    long zScaled = map(zRaw, RawMin, RawMax, -3000, 3000); // +-3g z-axis

    // Re-scale to fractional Gs
    float xAccel = xScaled / 1000.0;
    float yAccel = yScaled / 1000.0;
    float zAccel = zScaled / 1000.0;

    Serial.print("X, Y, Z :: ");
    Serial.print(xRaw);
    Serial.print(", ");
    Serial.print(yRaw);
    Serial.print(", ");
    Serial.print(zRaw);
    Serial.print(" :: ");
    Serial.print(xAccel, 2);
    Serial.print("G, ");
    Serial.print(yAccel, 2);
    Serial.print("G, ");
    Serial.print(zAccel, 2);
    Serial.println("G");

    delay(200); //200
}

// Take samples and return the average
int ReadAxis(int axisPin)
{
    long reading = 0;

    analogRead(axisPin);   // throw away first read
    delay(1);

    for (int i = 0; i < sampleSize; i++)
    {
        reading += analogRead(axisPin);
    }

    return reading / sampleSize;
}