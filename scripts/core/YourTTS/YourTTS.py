#!/usr/bin/env python3
import os
import typing

import numpy as np
import torch

import scipy.signal
import pyaudio

from TTS.config import load_config
from TTS.tts.models import setup_model
from TTS.utils.audio import AudioProcessor
from TTS.tts.utils.synthesis import synthesis
from TTS.tts.utils.speakers import SpeakerManager


class YourTTS:

    MODEL_PATH = f'{os.path.dirname(__file__)}/model/best_model.pth.tar'
    CONFIG_PATH = f'{os.path.dirname(__file__)}/model/config.json'
    TTS_LANGUAGES = f"{os.path.dirname(__file__)}/model/language_ids.json"
    TTS_SPEAKERS = f"{os.path.dirname(__file__)}/model/speakers.json"
    CONFIG_SE_PATH = f"{os.path.dirname(__file__)}/model/config_se.json"
    CHECKPOINT_SE_PATH = f"{os.path.dirname(__file__)}/model/SE_checkpoint.pth.tar"

    def __init__(self, ref_files, use_respeaker=False, use_cuda=False):
        if not os.path.exists(f'{os.path.dirname(__file__)}/model/'):
            os.system(f'mkdir {os.path.dirname(__file__)}/model')
            os.system(f"gdown --id 1-PfXD66l1ZpsZmJiC-vhL055CDSugLyP -O {os.path.dirname(__file__)}/model/config.json")
            os.system(f"gdown --id 1_Vb2_XHqcC0OcvRF82F883MTxfTRmerg -O {os.path.dirname(__file__)}/model/language_ids.json")
            os.system(f"gdown --id 1SZ9GE0CBM-xGstiXH2-O2QWdmSXsBKdC -O {os.path.dirname(__file__)}/model/speakers.json")
            os.system(f"gdown --id 1sgEjHt0lbPSEw9-FSbC_mBoOPwNi87YR -O {os.path.dirname(__file__)}/model/best_model.pth.tar")
            os.system(f"gdown --id  19cDrhZZ0PfKf2Zhr_ebB-QASRw844Tn1 -O {os.path.dirname(__file__)}/model/config_se.json")
            os.system(f"gdown --id   17JsW6h6TIh7-LkU2EvB_gnNrPcdBxt7X -O {os.path.dirname(__file__)}/model/SE_checkpoint.pth.tar")

        if use_respeaker:
            p = pyaudio.PyAudio()
            for i in range(p.get_device_count()):
                if str(p.get_device_info_by_index(i)['name'].encode('utf-8')).lower().find('respeaker') >= 0:
                    self.audio_device_idx = i
            p.terminate()
        else:
            self.audio_device_idx = 0


        self.use_cuda = use_cuda
        self.config = load_config(self.__class__.CONFIG_PATH)
        self.config.model_args['d_vector_file'] = self.__class__.TTS_SPEAKERS
        self.config.model_args['use_speaker_encoder_as_loss'] = False
        self.audio_processor = AudioProcessor(**self.config.audio)
        self.model = setup_model(self.config)
        self.model.language_manager.set_language_ids_from_file(self.__class__.TTS_LANGUAGES)
        self.cp = torch.load(self.__class__.MODEL_PATH, map_location=torch.device('cpu'))
        self.model_weights = self.cp['model'].copy()
        for key in list(self.model_weights.keys()):
            if 'speaker_encoder' in key:
                del self.model_weights[key]
        self.model.load_state_dict(self.model_weights)
        self.model.eval()
        if self.use_cuda:
            self.model = self.model.cuda()
        self.SE_speaker_manager = SpeakerManager(encoder_model_path=self.__class__.CHECKPOINT_SE_PATH,
                                                 encoder_config_path=self.__class__.CONFIG_SE_PATH,
                                                 use_cuda=self.use_cuda)
        self.reference_files = ref_files
        self.reference_emb = self.SE_speaker_manager.compute_d_vector_from_clip(self.reference_files)

        self.language_id = 0

    def say(self, text: typing.AnyStr):
        wav, alignment, _, outputs = synthesis(
            self.model,
            text,
            self.config,
            self.use_cuda,
            self.audio_processor,
            speaker_id=None,
            d_vector=self.reference_emb,
            style_wav=None,
            language_id=self.language_id,
            enable_eos_bos_chars=self.config.enable_eos_bos_chars,
            use_griffin_lim=True,
            do_trim_silence=False
        ).values()
        p = pyaudio.PyAudio()
        channels = p.get_device_info_by_index(self.audio_device_idx)['maxOutputChannels']
        sample_rate = int(np.ceil(p.get_device_info_by_index(self.audio_device_idx)['defaultSampleRate']))
        stream = p.open(format=pyaudio.paFloat32,
                        channels=channels,
                        rate=sample_rate,
                        output=True,
                        output_device_index=self.audio_device_idx)
        wav = scipy.signal.resample(wav, int(len(wav) * (sample_rate / (16000.0 / channels)))).astype(np.float32)
        stream.write(wav.tostring())
        stream.stop_stream()
        stream.close()
        p.terminate()
