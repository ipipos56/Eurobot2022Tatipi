from client import RobotClient

cl = RobotClient()


def act(inp):
    print(inp)


cl.connect(host="192.168.175.169")

cl.set_reaction(act)

cl.send("balish")

cl.send({"st":"1","mes":"Good","stop":"0","an":"45","dir":"12","t":"-150"})