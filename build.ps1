Remove-Item -Recurse -Force bld
$options = @('--configuration', 'Release', '-p:DebugType=embedded')
dotnet publish ./RadialFollow $options --framework net8.0 -o ./bld


Write-Host -NoNewLine 'Press any key to continue...';
$null = $Host.UI.RawUI.ReadKey('NoEcho,IncludeKeyDown');
