#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "log.h"

#define FREE_CHECK_NULL(x) do {                                                \
  if (!x) {                                                                    \
    free(x);                                                                   \
  }                                                                            \
} while(0)                                                                     \


//Count is first 4 bytes
//Size is size of image 

//short tmp = n;
//fread( &tmp, sizeof( tmp ), n, file);

void ReadVecHeader(FILE *file, int *count, int *size) {
  // Read first four bytes of vec file
  fread(count, sizeof(int), 1, file);
  LOG_INFO("count: %d", *count);

  // Read next four bytes into size
  fread(size, sizeof(int), 1, file);
  LOG_INFO("size: %d", *size);

  // Read in two shorts
  int tmp;
  fread(&tmp, sizeof(int), 1, file);
}

void WriteVecHeader(FILE *file, int count, int size) {
  // Read first four bytes of vec file
  fwrite(&count, sizeof(count), 1, file);

  // Read next four bytes into size
  fwrite(&size, sizeof(size), 1, file);

  // Read in two shorts
  int tmp = 0;
  fwrite(&tmp, sizeof(tmp), 1, file);
}

// ReadVecSample: 
// Description: Read a single sample from a file and store
//              information into data. Data is a pre allocated
//              pointer.
// Number of bytes in image = size * size of short
void ReadVecSample(FILE *file, int size, unsigned char *data) {
  fread(data, sizeof(short)*size+1, 1, file);
}

void WriteVecSample(FILE *file, int size, unsigned char *data) {
  fwrite(data, sizeof(short)*size+1, 1, file);

}

void TransferSamples(FILE *file_input, FILE *file_output, int count, int size) {
  unsigned char *data;
  data = (unsigned char*)malloc(sizeof(short) * size+1);

  // Takes all samples from one file and puts into output file
  for (int x=0; x < count; x++) {
    ReadVecSample(file_input, size, data);
    WriteVecSample(file_output, size, data);
  }
  free(data);
}

void WriteCombinedHeader(FILE *file_output,
    int count_1, int count_2, int size) {
  
  int count_output = count_1 + count_2;
  WriteVecHeader(file_output, count_output, size);
}

int main(int argc, char *argv[]) {
  int error_flag = false, opt;
  char *filename_a = NULL, *filename_b = NULL, *output = NULL;
  while ((opt = getopt(argc, argv, "a:b:c:")) != -1) {
    switch (opt) {
      case 'a':
        filename_a = strdup(optarg);
        break;
      case 'b':
        filename_b = strdup(optarg);
        break;
      case 'c':
        output = strdup(optarg);
        break;
      default:
        printf(
          "Usage: %s [-a <Input Vec file>]"
          " [-b <Input Vec file>]"
          " [-c <Output vec file>]\n", argv[0]);
        exit(EXIT_FAILURE);
    }
  }
  if (!filename_a) {
    printf("ERROR: Input a is NULL\n");
    error_flag = true;
  }
  if (!filename_b) {
    printf("ERROR: Input b is NULL\n");
    error_flag = true;
  }
  if (!output) {
    printf("ERROR: Output is NULL\n");
    error_flag = true;
  }
  if (error_flag) {
    FREE_CHECK_NULL(filename_a);
    FREE_CHECK_NULL(filename_b);
    FREE_CHECK_NULL(output);
    return -1;
  }
  LOG_INFO("Input A: %s, Input B: %s, Output: %s",
    filename_a, filename_b, output);

  // Read Headers from the two input vec files
  FILE *file_a      = fopen(filename_a, "rb");
  LOG_INFO("File A path: %s, %d", filename_a, file_a);
  FILE *file_b      = fopen(filename_b, "rb");
  LOG_INFO("File B path: %s, %d", filename_b, file_b);
  FILE *output_file = fopen(output, "wb");
  LOG_INFO("File C path: %s, %d", output, output_file);
  int count_a, count_b, size_a, size_b;
  //Read Header from file_a
  ReadVecHeader(file_a, &count_a, &size_a);
  //Read Header from file_b
  ReadVecHeader(file_b, &count_b, &size_b);

  LOG_INFO("Count A: %d, Count B: %d, Size A: %d, Size B: %d", count_a, 
    count_b, size_a, size_b);

  // Check if image size is the same
  if (size_a != size_b) {
    printf("ERROR: Input vec files have different image size\n");
    FREE_CHECK_NULL(filename_a);
    FREE_CHECK_NULL(filename_b);
    FREE_CHECK_NULL(output);
    return -1;
  }
  // Generate Output combined vec file header
  WriteCombinedHeader(output_file, count_a, count_b, size_a);
  // Copy vec samples from first vec file
  TransferSamples(file_a, output_file, count_a, size_a);
  // copy vec samples from second vec file
  TransferSamples(file_b, output_file, count_b, size_b);

  fclose(file_a);
  fclose(file_b);
  fclose(output_file);

  FREE_CHECK_NULL(filename_a);
  FREE_CHECK_NULL(filename_b);
  FREE_CHECK_NULL(output);
  return 0;
}
