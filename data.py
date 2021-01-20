

org = open("experiment.txt", "r")
proces = open("EX.csv", "w")

lines = org.readlines() 
  
count = 0
# Strips the newline character 
i = 0
sum1=0
sum2=0
for line in lines: 
	if line.find("Mean distance form")>0:
		i+=1
		print(line.split(" "))
		sum1 += float(line.split(" ")[13].split("/")[0])
		sum2 += float(line.split(" ")[21])
		if i == 7:
			sum1/=7
			sum2/=7
			i=0
			x = str(int(sum1))+" "+str(int(sum2))
			sum1 = sum2 = 0
			print(x)
			proces.write(x+"\n")
