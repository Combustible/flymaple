import matplotlib as mpl

mpl.use("qt4agg")

import matplotlib.pyplot as plt
import numpy
import sys
import time

ARRAYSIZE = 200
HISTSIZE = 30

UPDATE_FREQ = 3
update_count = 0



sig = numpy.zeros((6,ARRAYSIZE), dtype=float)
w = numpy.zeros((6,ARRAYSIZE), dtype=float)
v = numpy.zeros((6,ARRAYSIZE), dtype=float)
k = numpy.zeros((3,ARRAYSIZE), dtype=float)
m = numpy.zeros((4,ARRAYSIZE), dtype=int)
perf = numpy.zeros((ARRAYSIZE), dtype=float)
base = numpy.zeros((ARRAYSIZE), dtype=int)
sig_start = 30
w_start = 60
v_start = 90
k_start = 120
m_start = 150



x = numpy.linspace(-6, 0, HISTSIZE)

# Figure 1 components
# f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
ax1 = plt.subplot(3,2,1)
ax2 = plt.subplot(3,2,3)
ax3 = plt.subplot(3,2,5)

line_sig = []
line_w = []
line_v = []
line_k = []
line_m = []

for i in range(0, 6):
    tempitem, = ax1.plot(x, sig[i][sig_start:sig_start + HISTSIZE])
    line_sig.append(tempitem)
    tempitem, = ax2.plot(x, w[i][w_start:w_start + HISTSIZE])
    line_w.append(tempitem)
    tempitem, = ax3.plot(x, v[i][v_start:v_start + HISTSIZE])
    line_v.append(tempitem)

ax1.set_title('Hidden Layer Weights (sigma)')
ax2.set_title('Actor Weights (w)')
ax3.set_title('Critic Weights (v)')

# Figure 2 components
# f2, (ax4, ax5, ax6) = plt.subplots(3, sharex=True)
ax4 = plt.subplot(3,2,2)
ax5 = plt.subplot(3,2,4)
ax6 = plt.subplot(3,2,6)

for i in range(0, 3):
    tempitem, = ax4.plot(x, k[i][k_start:k_start + HISTSIZE])
    line_k.append(tempitem)
for i in range(0, 4):
    tempitem, = ax5.plot(x, m[i][m_start:m_start + HISTSIZE])
    line_m.append(tempitem)
tempitem, = ax5.plot(x, base[m_start:m_start + HISTSIZE])
line_m.append(tempitem)
line_perf, = ax6.plot(x, perf[m_start:m_start + HISTSIZE])

ax4.set_title('Output Parameters (k)')
ax5.set_title('Motor speeds (m)')
ax6.set_title('Performance metric')


# Fine-tune figure; make subplots close to each other and hide x ticks for
# all but bottom plot.
# f.subplots_adjust(hspace=0)
# plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)

#~ print sig.shape[0]

#~ exit()

plt.show(block=False)

plt.pause(2)


minrange = numpy.array([0,0,0,0,0,0], dtype=float)
maxrange = numpy.array([1,1,1,1,1,0.01], dtype=float)


#~ while sys.stdin:
    #~ ready = select.select(read_list, [], [], timeout)[0]
    #~ if not ready:
        #~ plt.pause(0.001)
    #~ else:
        #~ line = sys.stdin.readline()

for line in sys.stdin:
    try:
        if (line.startswith('sig_')):
            line = line[len('sig_'):]
            index = int(line[0])
            val = float(line[2:])

            if (index == 0):
                sig_start = sig_start + 1

                if (sig_start + HISTSIZE - 1 >= ARRAYSIZE):
                    sig_start = 0
                    for i in range(0, sig.shape[0]):
                        sig[i][sig_start:sig_start + HISTSIZE - 1] = sig[i][ARRAYSIZE - HISTSIZE + 1: ARRAYSIZE]

            sig[index][sig_start + HISTSIZE - 1] = val

            minrange[0] = min(minrange[0], val - 0.1)
            maxrange[0] = max(maxrange[0], val + 0.1)

        elif (line.startswith('w_')):
            line = line[len('w_'):]
            index = int(line[0])
            val = float(line[2:])

            if (index == 0):
                w_start = w_start + 1

                if (w_start + HISTSIZE - 1 >= ARRAYSIZE):
                    w_start = 0
                    for i in range(0, w.shape[0]):
                        w[i][w_start:w_start + HISTSIZE - 1] = w[i][ARRAYSIZE - HISTSIZE + 1: ARRAYSIZE]

            w[index][w_start + HISTSIZE - 1] = val

            minrange[1] = min(minrange[1], val - 0.1)
            maxrange[1] = max(maxrange[1], val + 0.1)

        elif (line.startswith('v_')):
            line = line[len('v_'):]
            index = int(line[0])
            val = float(line[2:])

            if (index == 0):
                v_start = v_start + 1

                if (v_start + HISTSIZE - 1 >= ARRAYSIZE):
                    v_start = 0
                    for i in range(0, v.shape[0]):
                        v[i][v_start:v_start + HISTSIZE - 1] = v[i][ARRAYSIZE - HISTSIZE + 1: ARRAYSIZE]

            v[index][v_start + HISTSIZE - 1] = val

            minrange[2] = min(minrange[2], val - 0.1)
            maxrange[2] = max(maxrange[2], val + 0.1)

        elif (line.startswith('k_')):
            line = line[len('k_'):]
            index = int(line[0])
            val = float(line[2:])

            if (index == 0):
                k_start = k_start + 1

                if (k_start + HISTSIZE - 1 >= ARRAYSIZE):
                    k_start = 0
                    for i in range(0, k.shape[0]):
                        k[i][k_start:k_start + HISTSIZE - 1] = k[i][ARRAYSIZE - HISTSIZE + 1: ARRAYSIZE]

            k[index][k_start + HISTSIZE - 1] = val

            minrange[3] = min(minrange[3], val - 0.1)
            maxrange[3] = max(maxrange[3], val + 0.1)

        elif (line.startswith('m_')):
            line = line[len('m_'):]
            index = int(line[0])
            val = int(line[2:])

            if (index == 0):
                m_start = m_start + 1

                if (m_start + HISTSIZE - 1 >= ARRAYSIZE):
                    m_start = 0
                    for i in range(0, m.shape[0]):
                        m[i][m_start:m_start + HISTSIZE - 1] = m[i][ARRAYSIZE - HISTSIZE + 1: ARRAYSIZE]
                        base[m_start:m_start + HISTSIZE - 1] = base[ARRAYSIZE - HISTSIZE + 1: ARRAYSIZE]
                        perf[m_start:m_start + HISTSIZE - 1] = perf[ARRAYSIZE - HISTSIZE + 1: ARRAYSIZE]

            m[index][m_start + HISTSIZE - 1] = val

        elif (line.startswith('perf_')):
            val = float(line[len('perf_'):])
            perf[m_start + HISTSIZE - 1] = val

            maxrange[5] = max(maxrange[5], val)

        elif (line.startswith('base_')):
            val = int(line[len('base_'):])
            base[m_start + HISTSIZE - 1] = val

            minrange[4] = val - 400
            maxrange[4] = val + 400

            update_count += 1


            if (update_count >= UPDATE_FREQ):
                update_count = 0

                maxrange[5] = numpy.amax(perf)
                if (numpy.amax(perf) * 5 < maxrange[5]):
                    maxrange[5] = maxrange[5] * 3/4
                if (numpy.amax(v) * 5 < maxrange[2]):
                    maxrange[2] = maxrange[2] * 3/4

                # Update y data for all graphs
                for i in range(0, 6):
                    line_sig[i].set_ydata(sig[i][sig_start:sig_start + HISTSIZE])
                    line_w[i].set_ydata(w[i][w_start:w_start + HISTSIZE])
                    line_v[i].set_ydata(v[i][v_start:v_start + HISTSIZE])
                for i in range(0, 3):
                    line_k[i].set_ydata(k[i][k_start:k_start + HISTSIZE])
                for i in range(0, 4):
                    line_m[i].set_ydata(m[i][m_start:m_start + HISTSIZE])
                line_m[4].set_ydata(base[m_start:m_start + HISTSIZE])
                line_perf.set_ydata(perf[m_start:m_start + HISTSIZE])

                # Update y ranges for all graphs
                ax1.set_ylim([minrange[0], maxrange[0]])
                ax2.set_ylim([minrange[1], maxrange[1]])
                ax3.set_ylim([minrange[2], maxrange[2]])
                ax4.set_ylim([minrange[3], maxrange[3]])
                ax5.set_ylim([minrange[4], maxrange[4]])
                ax6.set_ylim([minrange[5], maxrange[5]])

                # Redraw the graphs
                plt.draw()

            # Pause to allow window events to be processed
            plt.pause(0.001)

    except ValueError:
        noop = 1
        #~ print "Noop"







#~ print "phi"
#~ print phi
#~ print "sig"
#~ print sig
#~ print "w"
#~ print w
#~ print "v"
#~ print v
#~ print "k"
#~ print k
#~ print "m"
#~ print m
#~ print "perf"
#~ print perf
#~ print "base"
#~ print base

