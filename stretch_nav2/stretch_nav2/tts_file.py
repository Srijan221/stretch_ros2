import openai
import speech_recognition as sr
import pyttsx3
import time
from gtts import gTTS
import os
import pygame
import io

class  GTTSWrapper:
    def __init__(self, lang='en', slow = False):
        self.lang = lang
        self.slow = slow
        pygame.mixer.init()

    def say(self,text):
        tts = gTTS(text=text, lang=self.lang, slow=self.slow)
        audio = io.BytesIO()
        tts.write_to_fp(audio)
        audio.seek(0)
        pygame.mixer.music.load(audio,"mp3")

        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            time.sleep(1)
    def set_language(self, lang):
        self.lang = lang
    def set_speed(self, slow):
        self.slow = slow
    def stop(self):
        pygame.mixer.music.stop()

class GPT4SpeechProcessor:
    def __init__(self, api_key = 'insert api key here', prefeeded_words = None, logger = None):
        self.api_key = api_key
        self.prefeeded_words = prefeeded_words
        # logger.info(prefeeded_words)
        self.engine = pyttsx3.init()
        self.set_voice_params()
        self.logger = logger
        openai.api_key = self.api_key

    def set_voice_params(self):
        voices = self.engine.getProperty("voices")
        # self.engine.setProperty('voice', voices[0].id)
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 1.0)

    def recognize_speech(self):
        # Initialize recognizer class (for recognizing the speech)
        recognizer = sr.Recognizer()

        # Capture the audio from the microphone
        with sr.Microphone() as source:
            # self.speak_text("Can you tell me - Where do you want to go?")
            self.logger.info("Listening...")
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)
        try:
            # Recognize speech using Google Speech Recognition
            speech_text = recognizer.recognize_google(audio)
            self.logger.info(f"You said: {speech_text}")
            return speech_text
        except sr.UnknownValueError:
            self.logger.info("Sorry, I could not understand the audio")
            return None
        except sr.RequestError as e:
            self.logger.info(f"Could not request results; {e}")
            return None

    def process_with_gpt4(self, speech_text):
        # Use GPT-4 API to process the speech and match predefined words
        response = openai.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are an assistant that identifies specific keywords from speech."},
                {"role": "user", "content": f"Here are some keywords to look for: {', '.join(self.prefeeded_words)}. Now, analyze the following text and tell me which keywords are present: {speech_text}. If there are no keywords, say 'NONE found' "}
            ]
        )
        # dean's office
        gpt4_response = response.choices[0].message.content
        self.logger.info(f"GPT-4 response: {gpt4_response}")
        
        # Extract matched words from GPT-4 response

        matched_words = [word for word in self.prefeeded_words if word in gpt4_response.lower()]
        return matched_words

    def speak_text(self, text):
        # Use pyttsx3 to convert text to speech
        self.engine.say(text)
        self.engine.runAndWait()
        # tts = GTTSWrapper()
        # tts.say(text)
        # tts.save('tts.mp3')
        # os.system('xdg-open tts.mp3')
        # time.sleep(1)

    def process_speech(self):
        # Recognize speech from the microphone
        speech_text = self.recognize_speech()

        if speech_text:
            if 'stop' in speech_text.lower():
                self.speak_text("Okay, I'll Stop now, Thankyou for traveling with me")
                # time.sleep(4)
                return 1            
            # Process the recognized speech with GPT-4 for matching prefeeded words
            matched_words = self.process_with_gpt4(speech_text)
            if matched_words:
                # Speak the matched words or generate a custom response
                response_text = f"I heard {', '.join(matched_words)}. Let's Start!"
                self.logger.info(response_text)
                self.speak_text(response_text)
                return matched_words
            else:
                self.logger.info("No prefeeded words matched.")
                self.speak_text(f"I'm sorry, I can't find this place, can you please let me know from places like the {self.prefeeded_words}")
                return 0
        else:
            self.logger.info("No speech detected.")
            self.speak_text("I didn't hear anything.")
            return 0

# Example usage
if __name__ == "__main__":
    api_key = 'insert api key here'
    prefeeded_words = ["hello", "world", "test", "speech"]
    processor = GPT4SpeechProcessor(api_key, prefeeded_words)
    processor.process_speech()

