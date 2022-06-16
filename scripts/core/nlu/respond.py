import yaml
import random


class Respond:

    def __init__(self, assistant):
        self.assistant = assistant
        self.yaml = []
        self.respond = {"intent": {}, "cannot_understand": []}
        self.__session = None
        self.__missing_slots = []

    @staticmethod
    def __say_respond(data):
        respond = {"type": "say", "texts": {}}
        for s in data["texts"]:
            a = set()
            i = s.find("[")
            while i != -1:
                a.add(s[i + 1:s.find("]", i + 1)])
                i = s.find("[", i + 1)
            a = frozenset(a)
            if a not in respond["texts"].keys():
                respond["texts"][a] = []
            respond["texts"][a].append(s)
        return respond

    @staticmethod
    def __get_no_slot_respond(respond):
        new_respond = []
        for res in respond:
            if res["type"] == "say":
                new_respond.append({"type": "say", "text": random.choice(res["texts"])})
            else:
                new_respond.append(res)
        return new_respond

    def __get_respond_from_session(self, session):
        respond = []
        for res in self.respond["intent"][session.intent_name]["respond"]:
            if res["type"] == "say":
                slot_names = set()
                for slot_name in self.respond["intent"][session.intent_name]["slots"].keys():
                    if slot_name not in session.missing_slots:
                        slot_names.add(slot_name)
                if frozenset(slot_names) in res["texts"]:
                    text = random.choice(res["texts"][frozenset(slot_names)])
                    for slot_name in slot_names:
                        index = self.respond["intent"][session.intent_name]["slots"][slot_name]["index"]
                        text = text.replace(f"[{slot_name}]", session.parse_result['slots'][index]['value']['value'])
                    respond.append({"type": "say", "text": text})
                else:
                    return self.__get_no_slot_respond(self.respond["cannot_understand"])
            else:
                respond.append(res)
        return respond

    def from_yaml(self, filename):
        with open(filename, "r") as file:
            self.yaml = [data for data in yaml.safe_load_all(file)]
        for data in self.yaml:
            if data["type"] == "intent":
                self.respond["intent"][data["name"]] = {"slots": {}, "respond": []}
                if "slots" in data.keys():
                    for i, slot in enumerate(data["slots"]):
                        self.respond["intent"][data["name"]]["slots"][slot["name"]] = {"index": i, "optional": "optional" in slot.keys() and slot["optional"]}
                        if not self.respond["intent"][data["name"]]["slots"][slot["name"]]["optional"]:
                            self.respond["intent"][data["name"]]["slots"][slot["name"]]["missing_slot_respond"] = slot["missing_slot_respond"]
                for res in data["respond"]:
                    if res["type"] == "say":
                        self.respond["intent"][data["name"]]["respond"].append(self.__say_respond(res))
                    else:
                        self.respond["intent"][data["name"]]["respond"].append(res)
            if data["type"] == "cannot_understand":
                self.respond["cannot_understand"] = data["respond"]

    def get_respond(self, text):
        if len(self.__missing_slots) == 0:
            session = self.assistant.session.request(text)
            if session.intent_name in self.respond["intent"].keys():
                for slot_name in self.respond["intent"][session.intent_name]["slots"].keys():
                    if not self.respond["intent"][session.intent_name]["slots"][slot_name]["optional"] and slot_name in session.missing_slots:
                        self.__missing_slots.append(slot_name)
                if len(self.__missing_slots) > 0:
                    self.__session = session
                    return self.__get_no_slot_respond(self.respond["intent"][session.intent_name]["slots"][self.__missing_slots[0]]["missing_slot_respond"])
                return self.__get_respond_from_session(session)
            return self.__get_no_slot_respond(self.respond["cannot_understand"])
        try:
            self.__session.set_slot(self.__missing_slots[0], text)
            self.__missing_slots.pop(0)
            if len(self.__missing_slots) == 0:
                session = self.__session
                self.__session = None
                return self.__get_respond_from_session(session)
        except ValueError:
            return self.__get_no_slot_respond(self.respond["cannot_understand"])
