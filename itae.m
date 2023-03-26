function p=itae(n,wn)

switch(n)
    case(1)
        p=-wn;
    case(2)
        p=roots([1 1.4*wn wn^2])';
    case(3)
        p=roots([1 1.75*wn 2.15*wn^2 wn^3])';
    case(4)
        p=roots([1 2.1*wn 3.4*wn^2 2.7*wn^3 wn^4])';
    case(5)
        p=roots([1 2.8*wn 5.0*wn^2 5.5*wn^3 3.4*wn^4 wn^5])';
    case(6)
        p=roots([1 3.25*wn 6.6*wn^2 8.6*wn^3 7.45*wn^4 3.95*wn^5 wn^6])';
end
