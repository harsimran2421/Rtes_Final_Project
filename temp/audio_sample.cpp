#include </usr/include/festival/festival.h>

int main(int argc, char** argv)
{
  //EST_Wave wave; 
  int heap_size = 210000; 
  int load_init_files = 1; 

  festival_initialize(load_init_files, heap_size); 

  //festival_say_file("/etc/motd"); 

  //festival_eval_command("(voice_ked_diphone)"); 

  festival_say_text("hello world"); 

  //festival_text_to_wave("hello world", wave); 

  //wave.save("/tmp/wave.wav", "riff"); 

  //festival_wait_for_spooler(); 

  return 0; 
}
