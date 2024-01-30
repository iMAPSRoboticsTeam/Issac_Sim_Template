set mypath=%cd%

for /f "delims=" %%a in ('dir "%mypath:~0,-18%\exts" /on /ad /b') do @set company=%%a

@echo %company%

for /r "%mypath%" %%x in (*.py) do move "%%x" "%mypath:~0,-18%\exts\%company%\%company%"

cd ..
@RD /S /Q "%mypath%"
