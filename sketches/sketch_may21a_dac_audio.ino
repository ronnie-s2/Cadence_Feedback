#include <driver/i2s.h>
#include <math.h>

//
// I2S Pin Configuration
//
#define I2S_BCK 26
#define I2S_WS  25
#define I2S_DO  22

//
// Audio Settings
//
#define SAMPLE_RATE     44100
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT

//
// Metronome Settings
//
#define BPM             120
#define CLICK_FREQ      1200.0f
#define CLICK_MS        30

//
// Audio Buffer
//
#define BUFFER_SAMPLES  256

int16_t audioBuffer[BUFFER_SAMPLES * 2]; // stereo interleaved

bool clickLeft = true;

void setupI2S()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_DO,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

void generateClick(bool leftChannel)
{
    const int clickSamples =
        (SAMPLE_RATE * CLICK_MS) / 1000;

    float phase = 0.0f;
    float phaseInc =
        2.0f * PI * CLICK_FREQ / SAMPLE_RATE;

    int samplesRemaining = clickSamples;

    while (samplesRemaining > 0)
    {
        int chunk = min(samplesRemaining, BUFFER_SAMPLES);

        for (int i = 0; i < chunk; i++)
        {
            // Exponential decay envelope
            float env =
                expf(-5.0f * (float)i / clickSamples);

            float s =
                sinf(phase) * env;

            int16_t sample =
                (int16_t)(s * 12000);

            phase += phaseInc;

            // Stereo interleaved
            if (leftChannel)
            {
                audioBuffer[i * 2 + 0] = sample; // LEFT
                audioBuffer[i * 2 + 1] = 0;      // RIGHT
            }
            else
            {
                audioBuffer[i * 2 + 0] = 0;
                audioBuffer[i * 2 + 1] = sample;
            }
        }

        size_t bytesWritten;

        i2s_write(
            I2S_NUM_0,
            audioBuffer,
            chunk * 2 * sizeof(int16_t),
            &bytesWritten,
            portMAX_DELAY
        );

        samplesRemaining -= chunk;
    }
}

void silenceMs(int ms)
{
    int totalSamples =
        (SAMPLE_RATE * ms) / 1000;

    memset(audioBuffer, 0, sizeof(audioBuffer));

    while (totalSamples > 0)
    {
        int chunk = min(totalSamples, BUFFER_SAMPLES);

        size_t bytesWritten;

        i2s_write(
            I2S_NUM_0,
            audioBuffer,
            chunk * 2 * sizeof(int16_t),
            &bytesWritten,
            portMAX_DELAY
        );

        totalSamples -= chunk;
    }
}

void setup()
{
    setupI2S();
}

void loop()
{
    int beatPeriodMs = 60000 / BPM;

    generateClick(clickLeft);

    clickLeft = !clickLeft;

    silenceMs(beatPeriodMs - CLICK_MS);
}