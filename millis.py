import datetime

t0= datetime.datetime.now()
delay=5

while True:
	t=datetime.datetime.now()
	if (t-t0).seconds>delay:
		print "pasaron ",delay," segundos"
		t0=datetime.datetime.now()


