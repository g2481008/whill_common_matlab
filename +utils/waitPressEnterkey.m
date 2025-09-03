function [] = waitPressEnterkey(Stopper)
    waitTime = 1.e-5;
    while Stopper.UserData == -1
        pause(waitTime);
    end
end