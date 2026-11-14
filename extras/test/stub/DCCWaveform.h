#pragma once
enum class POWERMODE { OFF, ON, OVERLOAD };
struct DCCWaveform { int getCurrentmA(); void setPowerMode(POWERMODE); POWERMODE getPowerMode(); static DCCWaveform mainTrack, progTrack; };
