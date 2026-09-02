#pragma once
#include "config.h"

void rtcInit() {
  Rtc.Begin();
}

String rtcGetDateString() {
  RtcDateTime now = Rtc.GetDateTime();
  char dateString[11];
  snprintf(dateString, sizeof(dateString), "%04u-%02u-%02u", now.Year(), now.Month(), now.Day());
  return String(dateString);
}
