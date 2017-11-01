from decimal import Decimal
with open("train_states.txt") as f:
    s = f.readlines()
    ss = 0
    print len(s)
    for i in range(len(s)):
        ss += Decimal(s[i].split(',')[0])
    print  ss
