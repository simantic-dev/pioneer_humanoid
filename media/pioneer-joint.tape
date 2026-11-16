Output media/pioneer-joint.gif
Set Shell zsh
Set FontSize 15
Set Width 1280
Set Height 740
Set Padding 18
Set Theme "Catppuccin Mocha"
Set TypingSpeed 35ms
Env PS1 "$ "
Hide
Type 'cd "$(git rev-parse --show-toplevel)" && clear' Enter
Show
Type "sim run --scenario sim/scenario.yaml --usb-cdc-console dut.usb --timeout 3.5 --ascii 2>&1 \" Enter
Type "    | grep --line-buffered -E 'MOT:(sensor|PP)|Motor ready|canpeer'" Enter
Wait+Screen@180s /joint followed CAN command/
Sleep 3s
