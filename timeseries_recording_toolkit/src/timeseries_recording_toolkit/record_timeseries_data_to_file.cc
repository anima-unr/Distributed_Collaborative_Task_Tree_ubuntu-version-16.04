/*
timeseries_recording_toolkit
Copyright (C) 2015  Luke Fraser

timeseries_recording_toolkit is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

timeseries_recording_toolkit is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with timeseries_recording_toolkit.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "log.h"
#include "timeseries_recording_toolkit/record_timeseries_data_to_file.h"

#define BUFFER_SIZE 2048
#define MAX_BUFFER_SLEEP 1024
#define SLEEP_TIME 100
#define MAX_EXPO_FALLOFF 80

//  Swap macro  two swap the double buffer
#define SWAP(A_PTR, B_PTR) do {                                                \
  typeof(A_PTR) PTR = A_PTR;                                                   \
  A_PTR = B_PTR;                                                               \
  B_PTR = PTR;                                                                 \
} while (0)                                                                    \


namespace recording_toolkit {
typedef enum {
  SUCCESS = 1,
  NOT_RECORDING,
} error_recording;

PrintRecorder::PrintRecorder(uint32_t queue_size) {
  printing_thread = NULL;
  queue_size_ = queue_size;
  thread_queue_ptr_ = &queues[0];
  thread_queue_buffer_ptr_ = &queues[1];
}
PrintRecorder::~PrintRecorder() {
  if (printing_thread)
    delete printing_thread;
}

uint32_t PrintRecorder::RecordPrintf(const char *fmt, ...) {
  // Generate String to Add to file
  if (!recording_)
    return NOT_RECORDING;
  // Static variable to avoid reallocation on each call
  static char buffer[BUFFER_SIZE];

  // Generate buffer string to be pushed onto the queue
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, BUFFER_SIZE, fmt, args);
  va_end(args);

  // Push message to queue for processing
  thread_queue_ptr_->push(buffer);
  return SUCCESS;
}

void PrintRecorder::Worker(PrintRecorder *object) {
  uint32_t sleep;
  while (true) {
    sleep = object->RecordingWorker();
    boost::this_thread::sleep(boost::posix_time::milliseconds(sleep));
  }
}

uint32_t PrintRecorder::RecordingWorker() {
  static uint32_t count = 0;
  uint32_t result;
  uint32_t size = thread_queue_ptr_->size();
  if (size >= queue_size_) {
    SWAP(thread_queue_ptr_, thread_queue_buffer_ptr_);
    ProcessBufferQueue(thread_queue_buffer_ptr_);
    return SLEEP_TIME;
  } else if (size > 0) {
    count++;
    if (count > MAX_BUFFER_SLEEP) {
      SWAP(thread_queue_ptr_, thread_queue_buffer_ptr_);
      ProcessBufferQueue(thread_queue_buffer_ptr_);
      count = 0;
    }
    return SLEEP_TIME;
  } else {
    if (count < MAX_EXPO_FALLOFF) {count++;}
    return pow(1.1, count+1);
  }
}

void PrintRecorder::ProcessBufferQueue(std::queue<std::string> *buffer) {
  std::string cat_str = "";
  while (!buffer->empty()) {
    cat_str += buffer->front();
    // LOG_INFO("%s", buffer->front().c_str());
    buffer->pop();
  }
  printf("%s", cat_str.c_str());
}

uint32_t PrintRecorder::StartRecord() {
  printing_thread = new boost::thread(PrintRecorder::Worker, this);
  recording_ = true;
}
uint32_t PrintRecorder::StopRecord() {
  if (printing_thread) {
    printing_thread->interrupt();
    delete printing_thread;
    printing_thread = NULL;
  }
  recording_ = false;
}

void PrintRecorder::WaitUntilFinishedWriting() {
  if (printing_thread) {
    while (!(queues[0].empty() && queues[1].empty())) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
  }
}

FilePrintRecorder::FilePrintRecorder(
    const char* filename,
    uint32_t queue_size)
    : PrintRecorder(queue_size) {
  filename_ = filename;
  fout.open(filename);
}

FilePrintRecorder::~FilePrintRecorder() {
  fout.close();
}

void FilePrintRecorder::ProcessBufferQueue(std::queue<std::string> *buffer) {
  std::string cat_str = "";
  while (!buffer->empty()) {
    cat_str += buffer->front();
    // LOG_INFO("%s", buffer->front().c_str());
    buffer->pop();
  }
  fout.write(cat_str.c_str(), cat_str.length());
}
}  // namespace recording_toolkit
