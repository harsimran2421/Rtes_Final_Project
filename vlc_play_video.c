#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<string.h>
#include<stdbool.h>
#include<unistd.h>

#include<vlc/vlc.h>

int main()
{
  libvlc_instance_t  *my_instance;
  
  my_instance = libvlc_new(0, NULL);

  if(my_instance == NULL) {
    printf("There was an error initializing VLC\n");
    exit(1);
  } else {
    printf("VLC initialized successfully\n");
  }

  libvlc_media_t *my_media_file;
  my_media_file = libvlc_media_new_path(my_instance, "./videoplayback");

  libvlc_media_player_t *my_player;
  my_player = libvlc_media_player_new_from_media(my_media_file);

  libvlc_media_player_play(my_player);

  sleep(5);

  libvlc_media_release(my_media_file);
  libvlc_media_player_release(my_player);
  libvlc_release(my_instance);

  return 0;
}

