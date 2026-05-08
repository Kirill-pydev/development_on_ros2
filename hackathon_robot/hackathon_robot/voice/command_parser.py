"""Парсинг голосовых команд по схеме учебника (раздел NLP)."""


class CommandParser:
    """Извлекает действие и параметры из русской фразы."""

    def __init__(self) -> None:
        self.action_keywords = {
            'find_object': ['найди', 'найти', 'покажи', 'где'],
            'navigate_to': ['подъедь', 'поезжай', 'иди', 'едь'],
            'stop': ['стоп', 'останов', 'хватит', 'остановись'],
        }
        self.colors = {
            'красный': 'red',
            'синий': 'blue',
            'зелёный': 'green',
            'жёлтый': 'yellow',
            'оранжевый': 'orange',
        }
        self.objects = {
            'куб': 'cube',
            'кубик': 'cube',
            'кружка': 'cup',
            'человек': 'person',
        }

    def parse(self, text: str) -> dict:
        text_lower = text.lower().strip()
        action = self._detect_action(text_lower)
        params = self._extract_params(text_lower, action)
        return {'action': action, 'params': params}

    def _detect_action(self, text: str) -> str:
        for action, keywords in self.action_keywords.items():
            for keyword in keywords:
                if keyword in text:
                    return action
        return 'unknown'

    def _extract_params(self, text: str, action: str) -> dict:
        params: dict = {}
        for ru_color, en_color in self.colors.items():
            if ru_color in text:
                params['color'] = en_color
        for ru_object, en_object in self.objects.items():
            if ru_object in text:
                params['object'] = en_object
        return params

    def generate_response(self, command: dict) -> str:
        action = command['action']
        params = command['params']
        if action == 'find_object':
            obj = params.get('object', 'объект')
            color = params.get('color', '')
            if color:
                return f'Ищу {color} {obj}'
            return f'Ищу {obj}'
        if action == 'stop':
            return 'Останавливаюсь'
        if action == 'navigate_to':
            return 'Еду по команде'
        if action == 'unknown':
            return 'Не понял команду'
        return 'Команда принята'
