// Simple test program to verify your MIDI setup is working
#include <alsa/asoundlib.h>
#include <stdio.h>
#include <unistd.h>

// Your existing function (should work now)
unsigned int dev_midi_port_count(const char *path) {
    int card;
    int alsa_dev;

    if (sscanf(path, "/dev/snd/midiC%dD%d", &card, &alsa_dev) < 0) {
        printf("Failed to parse path: %s\n", path);
        return 0;
    }

    snd_ctl_t *ctl;
    char name[32];
    snd_rawmidi_info_t *info;
    int subs = 0;
    int subs_in = 0;
    int subs_out = 0;

    sprintf(name, "hw:%d", card);
    if (snd_ctl_open(&ctl, name, 0) < 0) {
        printf("Failed to open control for card %d\n", card);
        return 0;
    }

    snd_rawmidi_info_alloca(&info);
    snd_rawmidi_info_set_device(info, alsa_dev);

    snd_rawmidi_info_set_stream(info, SND_RAWMIDI_STREAM_INPUT);
    if (snd_ctl_rawmidi_info(ctl, info) >= 0) {
        subs_in = snd_rawmidi_info_get_subdevices_count(info);
    }

    snd_rawmidi_info_set_stream(info, SND_RAWMIDI_STREAM_OUTPUT);
    if (snd_ctl_rawmidi_info(ctl, info) >= 0) {
        subs_out = snd_rawmidi_info_get_subdevices_count(info);
    }
    snd_ctl_close(ctl);

    subs = subs_in > subs_out ? subs_in : subs_out;
    return subs;
}

// Simple test to open and send a MIDI message
int test_midi_device(const char* device_path, int port_index) {
    printf("Testing device: %s, port: %d\n", device_path, port_index);
    
    int card, alsa_dev;
    if (sscanf(device_path, "/dev/snd/midiC%dD%d", &card, &alsa_dev) != 2) {
        printf("  Invalid device path format\n");
        return -1;
    }
    
    char alsa_name[32];
    sprintf(alsa_name, "hw:%d,%d,%d", card, alsa_dev, port_index);
    printf("  Opening ALSA device: %s\n", alsa_name);
    
    snd_rawmidi_t *output = NULL;
    snd_rawmidi_t *input = NULL;
    
    // Try to open for output
    int err = snd_rawmidi_open(NULL, &output, alsa_name, SND_RAWMIDI_NONBLOCK);
    if (err < 0) {
        printf("  Failed to open output: %s\n", snd_strerror(err));
    } else {
        printf("  ✓ Output opened successfully\n");
        
        // Send a test note on message
        unsigned char note_on[] = {0x90, 60, 100}; // Note on, middle C, velocity 100
        ssize_t written = snd_rawmidi_write(output, note_on, 3);
        printf("  Sent note on message (%zd bytes)\n", written);
        
        snd_rawmidi_close(output);
    }
    
    // Try to open for input
    err = snd_rawmidi_open(&input, NULL, alsa_name, SND_RAWMIDI_NONBLOCK);
    if (err < 0) {
        printf("  Failed to open input: %s\n", snd_strerror(err));
    } else {
        printf("  ✓ Input opened successfully\n");
        snd_rawmidi_close(input);
    }
    
    return 0;
}

int main() {
    printf("=== MIDI Device Test ===\n\n");
    
    // Test available devices
    const char* test_devices[] = {
        "/dev/snd/midiC0D0",
        "/dev/snd/midiC0D1", 
        "/dev/snd/midiC0D2",
        "/dev/snd/midiC0D3",
        NULL
    };
    
    for (int i = 0; test_devices[i] != NULL; i++) {
        printf("Device: %s\n", test_devices[i]);
        unsigned int ports = dev_midi_port_count(test_devices[i]);
        printf("  Port count: %u\n", ports);
        
        if (ports > 0) {
            // Test first port
            test_midi_device(test_devices[i], 0);
        }
        printf("\n");
    }
    
    return 0;
}
