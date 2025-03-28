massive props to abstractqbit for making the original radial follow and making bezier interpolator, which i would recommend using with this over temporal resampler, because the cursor acts weird with temporal.

how to use for noobs

1. get otd branch 0.6.x (not master, not avalonia) source code from zip, unzip it
2. go to the otd source code folder and run build.ps1
3. if it doesnt work immediately then either enable scripts running using the solution shown to you by powershell or open powershell then run build.ps1
4. go to otd/bin, this is where your otd is
5. install a plugin that you will *not* use (important)
6. theres now a new folder, go to bin/userdata/plugins
7. create a folder named anything
8. insert adaptiveradialfollow.dll
9. close and reopen otd maybe
10. (important) make sure adaptive radial follow is the first active filter listed when you open otd daemon. it doesnt strictly have to be the *first* but i would definitely have it before bezier interpolator for example

if it is: profit

if its not:

11. save settings as x.json
12. open x.json with visual studio code
13. make all the adaptive follow stuff first in order {the whole fancy bracket}
14. ctrl+s
15. load x.json into otd
16. press save
17. profit

uses

vanilla: snaps immensely while providing minimal antichatter level smoothing when you're not trying to snap: you can make inner radius close to or equal to outer radius

relax: hangs where you snap, but still preserves cursor path coverage: inner radius can be small while outer radius is large

common problems

want more snap? increase radius or accel mult power.

cursor choppy on flow? increase radial mult power or velocity divisor

if you're stuttering, try using process lasso to make otd work on higher numbered cpu cores and force realtime priority

you can mess with the code and run build.ps1 on that folder if you want i just use dotnet build BUT its a noob guide
