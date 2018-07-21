import pickle
import matplotlib.pyplot as plt


f_eval = open("../data/ckpt/evaluation/"+"eval_"+"20180716_0541.ckpt", 'r')
eval = pickle.load(f_eval)
f_eval.close()

print "eval : ", len(eval)

t = []
for i in range(len(eval)-30):
	t.append(i+1)

plt.plot(eval[30:])
plt.xlabel('iteration')
plt.ylabel('average reward')
plt.title('convergence')
plt.show()
