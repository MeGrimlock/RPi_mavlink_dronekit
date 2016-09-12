import pyttsx

engine = pyttsx.init()
engine.setProperty('voice','english')
engine.setProperty('rate',120)

engine.say("Hello! Is it me youre looking for?")
engine.say(" ")


engine.setProperty('voice','french')
engine.say("Hakuna Matata, Mais quelle phrase magnifique ! Hakuna Matata, Quel chant fantastiiiique !")
engine.say(" ")


engine.setProperty('voice','spanish')
engine.say("Gracias")

engine.say(" ")

engine.runAndWait()
