These files are encypted using `aes-128-cbc`

Encryption Command:

```
openssl enc -aes-128-cbc -a -pbkdf2 -in PID_controller.py -out PID_controller.py.data
openssl enc -aes-128-cbc -a -pbkdf2 -in P_controller.py -out P_controller.py.data 
openssl enc -aes-128-cbc -a -pbkdf2 -in Ziegler–Nichols.py -out Ziegler–Nichols.py.data
openssl enc -aes-128-cbc -a -pbkdf2 -in Line_Follower_Testing.ipynb -out Line_Follower_Testing.ipynb.data 
```

To Decrypt Use these commands:

```
openssl enc -aes-128-cbc -d -a -pbkdf2 -in PID_controller.py.data -out PID_controller.py
openssl enc -aes-128-cbc -d -a -pbkdf2 -in P_controller.py.data -out P_controller.py 
openssl enc -aes-128-cbc -d -a -pbkdf2 -in Ziegler–Nichols.py.data -out Ziegler–Nichols.py
openssl enc -aes-128-cbc -d -a -pbkdf2 -in Line_Follower_Testing.ipynb.data -out Line_Follower_Testing.ipynb 
```

The Key has been mailed to the mentors.