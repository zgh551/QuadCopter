rd /Q /S USER\Flash
rd /Q /S USER\CpuRAM
rd /Q /S USER\ExtSRAM
del /Q USER\*.bak
del /Q USER\*.dep
del /Q USER\JLink*
del /Q USER\project.uvgui.*

del /Q project\EWARMv6\Project.dep
del /Q project\EWARMv6\Flash
del /Q project\EWARMv6\CpuRAM
del /Q project\EWARMv6\settings
rd  /Q /S project\EWARMv6\Flash
rd /Q /S project\EWARMv6\CpuRAM
rd /Q /S project\EWARMv6\settings

