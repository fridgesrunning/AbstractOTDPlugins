how to use for noobs
1. get otd source code from zip, unzip it
2. go to the otd source code folder and run build.ps1
3. if it doesnt work immediately then either enable scripts running using the solution shown to you by powershell or open powershell then run build.ps1
4. go to otd/bin, this is where your otd is
5. install any plugin
6. theres now a new folder, go to bin/userata/plugins
7. create a folder named anything
8. insert adaptiveradialfollow.dll
9. close and reopen otd maybe
10. make sure adaptive radial follow is the first filter listed when you open otd daemon. it doesnt really have to be the *first* but i would definitely have it before temporal resampler for example

if it is: profit

if its not:
11. save settings as x.json
12. open x.json with visual studio code
13. make all the adaptive follow stuff first in order {the whole fancy bracket}
14. ctrl+s
15. load x.json into otd
16. press save
17. profit

you can mess with the code and run build.ps1 on that folder if you want i just use dotnet build BUT its a noob guide