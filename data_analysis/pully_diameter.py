if __name__ == '__main__':
    pi = 3.14159265359
    start_turns = 6.295313358306885
    cm2_turns = 5.962521076202393
    cm4_turns = 5.631983280181885
    cm6_turns = 5.319576740264893
    cm8_turns = 4.961206912994385
    cm15_turns = 3.8318123817443848
    cm5_turns = 5.464502811431885
    cm7_turns = 5.130640506744385
    cm10_turns = 4.605709552764893
    cm3_turns = 5.799341678619385
    cm9_turns = 4.794674396514893

    val_cm2 = (20 / ((start_turns - cm2_turns) * pi))
    val_cm4 = (40 / ((start_turns - cm4_turns) * pi))
    val_cm6 = (60 / ((start_turns - cm6_turns) * pi))
    val_cm8 = (80 / ((start_turns - cm8_turns) * pi))
    val_cm15 = (150 / ((start_turns - cm15_turns) * pi))
    val_cm5 = (50 / ((start_turns - cm5_turns) * pi))
    val_cm7 = (70 / ((start_turns - cm7_turns) * pi))
    val_cm10 = (100 / ((start_turns - cm10_turns) * pi))
    val_cm3 = (30 / ((start_turns - cm3_turns) * pi))
    val_cm9 = (90 / ((start_turns - cm9_turns) * pi))

    samples = 10
    val_avg = (val_cm2 + val_cm4 + val_cm6 + val_cm8 + val_cm15 + val_cm5 + val_cm7 + val_cm10 + val_cm3 + val_cm9) / samples
    print(val_avg)

    print("#" * 40)
    radius = val_avg
    print((start_turns - cm2_turns) * pi * radius)
    print((start_turns - cm4_turns) * pi * radius)
    print((start_turns - cm6_turns) * pi * radius)
    print((start_turns - cm8_turns) * pi * radius)
    print((start_turns - cm15_turns) * pi * radius)

