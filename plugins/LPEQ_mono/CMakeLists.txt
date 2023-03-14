# CMake file for DISTRHO Plugins #
# ------------------------------ #

dpf_add_plugin(lpeqmono
  TARGETS jack lv2 vst2 vst3
  FILES_DSP
     LPEQMono.cpp
  FILES_UI
     LPEQMonoUI.cpp)

target_include_directories(
  lpeqmono PUBLIC ".")
