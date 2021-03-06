// Song synthesizer. rk4.c and OEC.new.dat are required for this program to work.
// Compile using: gcc synthesize.c rk4.c -lm -o synthesize
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "finch_void.c" 

void finch(int size, char *salida, char *envolvente);

int main(int argc, char *argv[]){
  int size_pitch_list;
  char *entrada; char *salida; char *envolvente;
  char *gestures;

if(argc < 4){
    printf("\nUsage:\n ./synthesize [Inputfile1] [Outputfile] [Inputfile2]\n[Inputfile1]: smooth_FF output file ('pitch list')\n[Outputfile]: Program output. Synthesized song \n[Inputfile3]: envelope.dat, Song envelope obtained by 'compute_FF'\n [Inputfile4]: a csv of the gestures (alpha and beta over time)");
    return 1;
  }


 entrada = argv[1];
 salida = argv[2];
 envolvente = argv[3];
 gestures = argv[4];

 size_pitch_list =  filesize(entrada, 3)-1;

	 int i,j,k, size_song;
	 double  nada;
      
	 double chi_freq[12500],chi_freq_min;

 double datos1[12500];
double datos2[12500];
double datos3[12500];
double datos4[12500];	
	 double aproximacion1[size_pitch_list], aproximacion_ant;
 double aproximacion2[size_pitch_list];
 double aproximacion3[size_pitch_list];
 double aproximacion4[size_pitch_list];
 double pitch_time[size_pitch_list];
 double pitch_value[size_pitch_list];
	 FILE *ptr1, *ptr2, *ptr3, *gesture_ptr;


ptr3 = fopen(salida,"w");

	 /*
ptr1=fopen("OEC.new.dat","r");
ptr2 = fopen(entrada,"r");
gesture_ptr = fopen(gestures, "w");

for(i=0;i<1500;i++){
fscanf(ptr1,"%lg %lg %lg %lg",&datos1[i],&datos2[i],&datos3[i],&datos4[i]);
// printf("%lg\n", datos1[i]);
 }
fclose(ptr1);

for(i=0;i<size_pitch_list;i++){
 fscanf(ptr2, "%lg\t%lg\t%lg\n",&nada, &pitch_time[i],&pitch_value[i]);
}

fclose(ptr2);


// Computes alpha and beta corresponding to the FF

for(i=0;i<size_pitch_list;i++){
 chi_freq_min=10000.;
  aproximacion1[i]=0; //alpha
  aproximacion2[i]=0; //beta
  aproximacion3[i]=0; //frequency
  aproximacion4[i]=0; //sci

  if(pitch_value[i]>10){


    for(k=0;k<1500;k++){
      if(fabs(datos3[k]-pitch_value[i])<chi_freq_min && (datos1[k] == -0.15)){
	chi_freq_min = fabs(datos3[k]-pitch_value[i]);
        aproximacion1[i]=datos1[k];aproximacion2[i]=datos2[k];aproximacion4[i]=datos4[k];aproximacion3[i]=datos3[k];
         }
     }
  }
  else{ aproximacion1[i]=0.15;aproximacion2[i]=0.15;aproximacion4[i]=0.;aproximacion3[i]=0.;}

}	

// write gestures
//fputs("time,alpha,beta\n", gesture_ptr);

for(i=0;i<size_pitch_list;i++){
	fprintf(gesture_ptr, "%lg,%lg,%lg\n", pitch_time[i], aproximacion1[i], aproximacion2[i]);
}
fclose(gesture_ptr);
	 */
for(i=0;i<size_pitch_list;i++) fprintf(ptr3,"%lg\t%lg\n",pitch_time[i], aproximacion2[i]);
 fclose(ptr3);
 finch(size_pitch_list, salida, envolvente);

return 0;
}
//------------------------------------------------------------------

int filesize(char name[], int ncol){

  int i, size = 1;
  char dato[20];
  FILE *datos;
  
  datos = fopen(name, "r");
  
  if(datos==NULL) {     
    printf("Missing file %s...\nExiting to system\n\n", name);
    exit(0);
  }
  
  while(!feof(datos)){
    for(i=1; i<=ncol; i++) fscanf(datos, "%s", &dato);
    size++;		
  }
  
  fclose(datos);
  return(size-1);
}


