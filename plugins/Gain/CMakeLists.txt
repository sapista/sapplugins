# CMake file for DISTRHO Plugins #
# ------------------------------ #

dpf_add_plugin(simplegain
  TARGETS jack lv2 vst2 vst3
  FILES_DSP
     GainPlugin.cpp
  FILES_UI
     GainPluginUI.cpp)

target_include_directories(
  simplegain PUBLIC ".")
