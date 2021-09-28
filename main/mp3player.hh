#pragma once
#include "AudioFileSourcePROGMEM.h"
#include "AudioFileSourceID3.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"
#include "AudioOutputNull.h"
#define TAG "PLAYER"

class MP3Player
{
public:
    void SetupInternalDAC()
    {
        out = new AudioOutputI2S(0, 1);
        //out = new AudioOutputNull();
    }

    void Play(const uint8_t *data, size_t length)
    {
        if(gen && gen->isRunning()) return;
        file = new AudioFileSourcePROGMEM(data, length);
        id3 = new AudioFileSourceID3(file);
        //id3->RegisterMetadataCB(MP3Player::MDCallback, (void*)"ID3TAG");
        gen = new AudioGeneratorMP3();
        gen->begin(id3, out);
        ESP_LOGI(TAG, "Player started");
    }

    void Loop(){
        if (gen && gen->isRunning()) {
            if (!gen->loop()){
                ESP_LOGI(TAG, "Player stopped");
                gen->stop();
                file->close();
                delete id3;
                delete gen;
                delete file;
                id3=NULL;
                gen=NULL;
                file=NULL;
                ESP_LOGI(TAG, "Player is prepared for next play");
            }
        }
        
    }
    static void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string)
    {
        (void)cbData;
        ESP_LOGI(TAG, "ID3 callback for: %s = %s'", type, string);
    }

private:
    AudioGeneratorMP3 *gen;
    AudioFileSourcePROGMEM *file;
    AudioOutput *out;
    AudioFileSourceID3 *id3;
};

#undef TAG