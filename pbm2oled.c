/* pbm2oled --- convert a bitmap from PBM to OLED 128x32    2015-03-31 */
/* Copyright (c) 2015 John Honniball. All rights reserved.             */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Size of OLED screen
#define MAXX 128
#define MAXY 32
#define MAXROWS 4  // 4 rows of bytes


// The frame buffer, 512 bytes
unsigned char Frame[MAXROWS][MAXX];

void writeOLED(const char name[], const int rows, const int wd);
int readPBM(const char name[]);

int main(const int argc, const char *const argv[])
{
   if (argc < 3) {
      fputs("Usage: pbm2oled <PBM_filename> <C_array_name>\n", stderr);
      exit(EXIT_FAILURE);
   }
   
   if (readPBM(argv[1]))
      writeOLED(argv[2], MAXROWS, MAXX);
      
   return (0);
}


/* writeOLED --- write the frame buffer into a C header file */

void writeOLED(const char name[], const int rows, const int wd)
{
   int x, y;
   
   printf("const uint8_t %s[%d * %d] = {\n", name, rows, wd);

   for (y = 0; y < MAXROWS; y++) {
      for (x = 0; x < MAXX; x++) {
         if ((x % 16) == 0)
            printf("  ");
            
         printf ("0x%02x", Frame[y][x]);
         
         if ((y == MAXROWS - 1) && (x == MAXX - 1))
            printf("\n");
         else if (x == 15)
            printf(", /* row %d */\n", y);
         else if ((x % 16) == 15)
            printf(",\n");
         else
            printf(", ");
      }
   }
  
   puts("};");
}


/* readPBM --- read a binary PBM file into the frame buffer in memory */

int readPBM(const char name[])
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

   /* Loop through PBM file, reading binary data */
   for (y = 0; y < ysize; y += 8) {
      for (x = 0; x < MAXX; x++)
         Frame[y / 8][x] = 0;

      for (i = 0; i < 8; i++) {
         if ((nb = fread(buf, sizeof (char), MAXX / 8, fp)) != (MAXX / 8)) {
            if ((nb == 0) && ((y + i) >= MAXY))
               memset(buf, 0, sizeof (buf));
            else {
               fprintf(stderr, "EOF!! (%d bytes, row %d)\n", nb, y + i);
               return (0);
            }
         }
         
         for (x = 0; x < MAXX; x++) {
            if ((buf[x / 8] & (1 << (7 - (x % 8)))) == 0)
               Frame[y / 8][x] |= (1 << i);
         }
      }
   }
   
   return (1);
}
