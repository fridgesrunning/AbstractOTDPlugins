Reinstall after September 10, 2025

massive props to abstractqbit for making the original radial follow and making bezier interpolator, which i would recommend using with this over temporal resampler, because the cursor acts weird with temporal.

how to use for noobs

1. go to otd
2. plugin manager
3. file top left
4. alternate source
5. owner: fridgesrunning
6. apply
7. profit

uses

vanilla: snaps immensely while providing minimal antichatter level smoothing when you're not trying to snap: you can make inner radius close to or equal to outer radius

if you use above ^ then i recommend trying adaptive bezier interpolator for slightly less delay

relax: hangs where you snap, but still preserves cursor path coverage: inner radius can be small while outer radius is large

common problems

want more snap? increase radius or accel mult power.

cursor choppy on flow? increase radial mult power or velocity divisor

if you're stuttering, try using process lasso to make otd work on higher numbered cpu cores and force realtime priority

you can mess with the code and run build.ps1 on that folder if you want i just use dotnet build BUT its a noob guide

