function value = InputParameter(txt, low, high)

    theta = input(txt);
    while theta<low || theta>high
        theta = input("This value is out of range, input again :");
    end
    value = theta;
end