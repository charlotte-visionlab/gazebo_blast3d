/* 
 * File:   WAVReader.h
 * Author: arwillis
 *
 * Created on March 21, 2024, 11:32 AM
 */

#ifndef WAVREADER_H
#define WAVREADER_H

#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using std::string;
using std::fstream;

typedef struct  WAV_HEADER{
    char                RIFF[4];        // RIFF Header      Magic header
    unsigned long       ChunkSize;      // RIFF Chunk Size  
    char                WAVE[4];        // WAVE Header      
    char                fmt[4];         // FMT header       
    unsigned long       Subchunk1Size;  // Size of the fmt chunk                                
    unsigned short      AudioFormat;    // Audio format 1=PCM,6=mulaw,7=alaw, 257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM 
    unsigned short      NumOfChan;      // Number of channels 1=Mono 2=Stereo                   
    unsigned long       SamplesPerSec;  // Sampling Frequency in Hz                             
    unsigned long       bytesPerSec;    // bytes per second 
    unsigned short      blockAlign;     // 2=16-bit mono, 4=16-bit stereo 
    unsigned short      bitsPerSample;  // Number of bits per sample      
    char                Subchunk2ID[4]; // "data"  string   
    unsigned long       Subchunk2Size;  // Sampled data length    

}wav_hdr; 

// Function prototypes 
int getFileSize(FILE *inFile); 

// https://stackoverflow.com/questions/13660777/c-reading-the-data-part-of-a-wav-file
//    // read chunk infos iteratively
//    ChunkInfo ch;
//    bool fmt_read = false;
//    bool data_read = false;
//    while(ifs.read((char*)(&ch), sizeof(ch))){
//
//        // if fmt chunk?
//        if (memcmp(ch.chunk_id, fmt_id, 4) == 0){
//            FmtChunk fmt;
//            ifs.read((char*)(&fmt), ch.chunk_size);
//            fmt_read = true;
//        }
//        // is data chunk?
//        else if(memcmp(ch.chunk_id, data_id, 4) == 0){
//            DataChunk dat_chunk(ch.chunk_size/sizeof(int16_t));
//            ifs.read((char*)dat_chunk.data, ch.chunk_size);
//            data_read = true;
//        }
//        // otherwise skip the chunk
//        else{
//            ifs.seekg(ch.chunk_size, ios_base::cur);
//        }
//    }
int readWAV(const char *filename) {
    wav_hdr wavHeader;
    FILE *wavFile;
    int headerSize = sizeof(wav_hdr),filelength = 0;
    string path;
    string answer;

    do{
        string input;
        string answer;

        const char* filePath;

        cout << "Pick wav file from the Windows Media File: ";
        cin >> input;
        cin.get();

        cout << endl;

        path = "C:\\Windows\\Media\\" + input + ".wav";
        filePath = path.c_str();

        wavFile = fopen( filePath , "r" );

        if(wavFile == NULL){
            printf("Can not able to open wave file\n");
            //exit(EXIT_FAILURE);
        }

        // fread(&wavHeader,headerSize,1,wavFile);
        size_t bytesRead = fread(&wavHeader, 1, headerSize, wavFile);
        filelength = getFileSize(wavFile);
        fclose(wavFile);

        cout << "File is                    :" << filelength << " bytes." << endl;

        cout << "RIFF header                :" << wavHeader.RIFF[0] 
                                                << wavHeader.RIFF[1] 
                                                << wavHeader.RIFF[2] 
                                                << wavHeader.RIFF[3] << endl;

        cout << "WAVE header                :" << wavHeader.WAVE[0] 
                                                << wavHeader.WAVE[1] 
                                                << wavHeader.WAVE[2] 
                                                << wavHeader.WAVE[3] 
                                                << endl;

        cout << "FMT                        :" << wavHeader.fmt[0] 
                                                << wavHeader.fmt[1] 
                                                << wavHeader.fmt[2] 
                                                << wavHeader.fmt[3]
                << endl;

        cout << "Data size                  :" << wavHeader.ChunkSize << endl;

        // Display the sampling Rate form the header
        cout << "Sampling Rate              :" << wavHeader.SamplesPerSec << endl;
        cout << "Number of bits used        :" << wavHeader.bitsPerSample << endl;
        cout << "Number of channels         :" << wavHeader.NumOfChan << endl;
        cout << "Number of bytes per second :" << wavHeader.bytesPerSec << endl;
        cout << "Data length                :" << wavHeader.Subchunk2Size << endl;
        cout << "Audio Format               :" << wavHeader.AudioFormat << endl;
        // Audio format 1=PCM,6=mulaw,7=alaw, 257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM 


        cout << "Block align                :" << wavHeader.blockAlign << endl;

        cout << "Data string                :" << wavHeader.Subchunk2ID[0] 
                                                << wavHeader.Subchunk2ID[1]
                                                << wavHeader.Subchunk2ID[2] 
                                                << wavHeader.Subchunk2ID[3] 
                                                << endl;

        cout << endl << endl << "Try something else? (y/n)";
        cin >> answer;
        //cin.get();
        cout << endl << endl;

    }while( answer == "y" );
    getchar();
    return 0;
}
// find the file size 

int getFileSize(FILE *inFile) {
    int fileSize = 0;
    fseek(inFile, 0, SEEK_END);

    fileSize = ftell(inFile);

    fseek(inFile, 0, SEEK_SET);
    return fileSize;
}

#endif /* WAVREADER_H */

