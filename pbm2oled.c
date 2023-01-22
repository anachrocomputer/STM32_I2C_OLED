/* pbm2oled --- convert a bitmap from PBM to OLED           2015-03-31 */
/* Copyright (c) 2015 John Honniball. All rights reserved.             */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Maximum size of OLED image
#define MAXX 256
#define MAXY 256
#define MAXROWS (MAXY / 8)  // Each row of bytes is 8 pixels

#define BYTES_PER_LINE  (16)  // Number of bytes of image data per line of source code


// The frame buffer, 512 bytes
unsigned char Frame[MAXROWS][MAXX];

void writeOLED(const char name[], const int rows, const int wd);
int readPBM(const char name[], int *const htp, int *const wdp);

int main(const int argc, const char *const argv[])
{
   int ht, wd;

   if (argc < 3) {
      fputs("Usage: pbm2oled <PBM_filename> <C_array_name>\n", stderr);
      exit(EXIT_FAILURE);
   }
   
   if (readPBM(argv[1], &ht, &wd))
      writeOLED(argv[2], ht / 8, wd);
      
   return (0);
}


/* writeOLED --- write the frame buffer into a C header file */

void writeOLED(const char name[], const int rows, const int wd)
{
   int x, y;
   
   printf("const uint8_t %s[%d][%d] = {\n", name, rows, wd);

   for (y = 0; y < rows; y++) {
      printf("   {\n");
      for (x = 0; x < wd; x++) {
         if ((x % BYTES_PER_LINE) == 0)
            printf("      ");
            
         printf ("0x%02x", Frame[y][x]);
         
         if (x == (wd - 1))
            printf("\n");
         else if (x == (BYTES_PER_LINE - 1))
            printf(", /* row %d */\n", y);
         else if ((x % BYTES_PER_LINE) == (BYTES_PER_LINE - 1))
            printf(",\n");
         else
            printf(", ");
      }
      
      if (y == (rows - 1))
         printf("   }\n");
      else
         printf("   },\n");
   }
  
   puts("};");
}


/* readPBM --- read a binary PBM file into the frame buffer in memory */

int readPBM(const char name[], int *const htp, int *const wdp)
{
   FILE *fp;
   char lin[256];
   int xsize, ysize;
   int i, j;
   int x, y;
   int ch;
   int nb;
   unsigned char buf[MAXX];
   
   if ((fp = fopen(name, "r")) == NULL) {
      perror(name);
      return (0);
   }

   /* Read PBM header */
   fgets (lin, sizeof (lin), fp);

   if (lin[0] != 'P') {
      fprintf(stderr, "Image file '%s' is not a PBM file\n", name);
      fclose(fp);
      return (0);
   }
   
   if (lin[1] != '4') {
      fprintf(stderr, "Image file '%s' not binary PBM file\n", name);
      fclose(fp);
      return (0);
   }

   fgets (lin, sizeof (lin), fp);
   
   if (lin[0] == '#')
      fgets (lin, sizeof (lin), fp);

   /* Read PBM X and Y size */
   
   sscanf (lin, "%d %d", &xsize, &ysize);
   
   if (xsize > MAXX) {
      fprintf(stderr, "Image width (%d) exceeds maximum (%d)\n", xsize, MAXX);
      fclose(fp);
      return (0);
   }

   if (ysize > MAXY) {
      fprintf(stderr, "Image height (%d) exceeds maximum (%d)\n", ysize, MAXY);
      fclose(fp);
      return (0);
   }
   
   /* Loop through PBM file, reading binary data */
   for (y = 0; y < ysize; y += 8) {
      for (x = 0; x < MAXX; x++)
         Frame[y / 8][x] = 0;

      for (i = 0; i < 8; i++) {
         if ((nb = fread(buf, sizeof (char), xsize / 8, fp)) != (xsize / 8)) {
            if ((nb == 0) && ((y + i) >= MAXY))
               memset(buf, 0, sizeof (buf));
            else {
               fprintf(stderr, "EOF!! (%d bytes, row %d)\n", nb, y + i);
               return (0);
            }
         }
         
         for (x = 0; x < xsize; x++) {
            if ((buf[x / 8] & (1 << (7 - (x % 8)))) == 0)
               Frame[y / 8][x] |= (1 << i);
         }
      }
   }
   
   if (htp != NULL)
      *htp = ysize;
   
   if (wdp != NULL)
      *wdp = xsize;
   
   return (1);
}
