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

#include <boost/timer/timer.hpp>
#include <stdio.h>
#include <iostream>
#include "timeseries_recording_toolkit/record_timeseries_data_to_file.h"

int main(int argc, char *argv[]) {
  recording_toolkit::FilePrintRecorder output_test("test.txt");

  output_test.StartRecord();

  boost::timer::cpu_timer timer;
  timer.start();
  for (int i = 0; i < 100000; ++i) {
    output_test.RecordPrintf("Hello this is me printing a number really fast %d\n", i);
  }
  timer.stop();
  output_test.WaitUntilFinishedWriting();
  output_test.StopRecord();
  std::cout << "First Timing: " << timer.format() << std::endl;

  // timer.start();
  // for (int i = 0; i < 100000; ++i) {
  //   printf("Hello this is me printing a number really slow %d\n", i);
  // }
  // timer.stop();
  // fout << "Second Timing: " << timer.format() << std::endl;
  return 0;
}