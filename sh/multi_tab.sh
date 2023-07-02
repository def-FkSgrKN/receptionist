tell application "Terminator"
    create window with default profile
    tell current window
        # tabの分割
        tell current session of current tab
            split vertically with default profile
        end tell

        # tab1の処理
        tell current session of current tab
            write text "ls"
            write text "ls"
        end tell

        # tab2の処理
        tell second session of current tab
            write text "ls"
            write text "ls
        end tell
    end tell
end tell