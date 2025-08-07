"""
Multi-language sentiment lexicons and language-specific processing
"""

from typing import Dict, Set, Optional, List, Tuple
from dataclasses import dataclass
import json
import logging

logger = logging.getLogger(__name__)


@dataclass
class LanguageConfig:
    """Configuration for a specific language"""
    code: str  # ISO 639-1 language code
    name: str
    rtl: bool = False  # Right-to-left text
    supported_features: List[str] = None
    confidence_modifier: float = 1.0  # Modifier for confidence scores


class MultiLanguageLexicons:
    """Multi-language sentiment lexicon manager"""
    
    def __init__(self):
        self.lexicons = {}
        self.language_configs = {}
        self.default_language = 'en'
        self._initialize_languages()
        self._load_lexicons()
        
        logger.info("MultiLanguageLexicons initialized")
    
    def _initialize_languages(self):
        """Initialize supported language configurations"""
        self.language_configs = {
            'en': LanguageConfig('en', 'English', False, ['lexicon', 'rules'], 1.0),
            'es': LanguageConfig('es', 'Spanish', False, ['lexicon'], 0.9),
            'fr': LanguageConfig('fr', 'French', False, ['lexicon'], 0.9),
            'de': LanguageConfig('de', 'German', False, ['lexicon'], 0.85),
            'ja': LanguageConfig('ja', 'Japanese', False, ['lexicon'], 0.8),
            'zh': LanguageConfig('zh', 'Chinese', False, ['lexicon'], 0.8),
            'ar': LanguageConfig('ar', 'Arabic', True, ['lexicon'], 0.7),
            'pt': LanguageConfig('pt', 'Portuguese', False, ['lexicon'], 0.9),
            'ru': LanguageConfig('ru', 'Russian', False, ['lexicon'], 0.8),
            'it': LanguageConfig('it', 'Italian', False, ['lexicon'], 0.9),
            'nl': LanguageConfig('nl', 'Dutch', False, ['lexicon'], 0.85),
            'sv': LanguageConfig('sv', 'Swedish', False, ['lexicon'], 0.85),
            'no': LanguageConfig('no', 'Norwegian', False, ['lexicon'], 0.85),
            'da': LanguageConfig('da', 'Danish', False, ['lexicon'], 0.85),
            'fi': LanguageConfig('fi', 'Finnish', False, ['lexicon'], 0.8),
            'pl': LanguageConfig('pl', 'Polish', False, ['lexicon'], 0.8),
            'ko': LanguageConfig('ko', 'Korean', False, ['lexicon'], 0.8),
            'hi': LanguageConfig('hi', 'Hindi', False, ['lexicon'], 0.7),
            'th': LanguageConfig('th', 'Thai', False, ['lexicon'], 0.7),
            'vi': LanguageConfig('vi', 'Vietnamese', False, ['lexicon'], 0.7)
        }
    
    def _load_lexicons(self):
        """Load sentiment lexicons for different languages"""
        # English lexicon (comprehensive)
        self.lexicons['en'] = {
            'positive': {
                'excellent', 'amazing', 'wonderful', 'fantastic', 'great', 'awesome',
                'brilliant', 'outstanding', 'superb', 'magnificent', 'perfect', 'incredible',
                'love', 'like', 'enjoy', 'appreciate', 'adore', 'cherish', 'treasure',
                'happy', 'joy', 'pleased', 'satisfied', 'delighted', 'thrilled', 'excited',
                'good', 'nice', 'fine', 'okay', 'decent', 'pleasant', 'lovely', 'beautiful',
                'valuable', 'useful', 'helpful', 'beneficial', 'advantageous', 'positive'
            },
            'negative': {
                'terrible', 'awful', 'horrible', 'disgusting', 'pathetic', 'useless',
                'bad', 'poor', 'disappointing', 'frustrating', 'annoying', 'irritating',
                'hate', 'dislike', 'detest', 'despise', 'loathe', 'abhor',
                'sad', 'angry', 'upset', 'disappointed', 'frustrated', 'annoyed',
                'broken', 'defective', 'faulty', 'damaged', 'worthless', 'inferior',
                'worst', 'horrible', 'dreadful', 'atrocious', 'appalling', 'negative'
            },
            'intensifiers': {
                'very', 'really', 'extremely', 'incredibly', 'absolutely', 'totally',
                'completely', 'entirely', 'quite', 'rather', 'pretty', 'fairly',
                'highly', 'deeply', 'truly', 'genuinely', 'remarkably', 'exceptionally'
            },
            'negations': {
                'not', 'no', 'never', 'none', 'nobody', 'nothing', 'neither',
                'nowhere', 'hardly', 'barely', 'rarely', 'seldom', 'scarcely'
            }
        }
        
        # Spanish lexicon
        self.lexicons['es'] = {
            'positive': {
                'excelente', 'increíble', 'maravilloso', 'fantástico', 'genial', 'estupendo',
                'brillante', 'extraordinario', 'magnífico', 'perfecto', 'bueno', 'bien',
                'amor', 'gustar', 'encantar', 'disfrutar', 'apreciar', 'adorar',
                'feliz', 'alegría', 'contento', 'satisfecho', 'emocionado', 'encantado',
                'útil', 'valioso', 'beneficioso', 'positivo', 'agradable', 'hermoso'
            },
            'negative': {
                'terrible', 'horrible', 'espantoso', 'asqueroso', 'patético', 'inútil',
                'malo', 'pobre', 'decepcionante', 'frustrante', 'molesto', 'irritante',
                'odiar', 'detestar', 'despreciar', 'aborrecer', 'disgustar',
                'triste', 'enojado', 'molesto', 'decepcionado', 'frustrado',
                'roto', 'defectuoso', 'dañado', 'inútil', 'inferior', 'negativo'
            },
            'intensifiers': {
                'muy', 'realmente', 'extremadamente', 'increíblemente', 'absolutamente',
                'totalmente', 'completamente', 'bastante', 'verdaderamente', 'sumamente'
            },
            'negations': {
                'no', 'nunca', 'nada', 'nadie', 'ningún', 'ninguna', 'jamás', 'ni'
            }
        }
        
        # French lexicon
        self.lexicons['fr'] = {
            'positive': {
                'excellent', 'incroyable', 'merveilleux', 'fantastique', 'génial', 'formidable',
                'brillant', 'extraordinaire', 'magnifique', 'parfait', 'bon', 'bien',
                'amour', 'aimer', 'adorer', 'apprécier', 'chérir', 'savourer',
                'heureux', 'joie', 'content', 'satisfait', 'ravi', 'enchanté',
                'utile', 'précieux', 'bénéfique', 'positif', 'agréable', 'beau'
            },
            'negative': {
                'terrible', 'affreux', 'horrible', 'dégoûtant', 'pathétique', 'inutile',
                'mauvais', 'pauvre', 'décevant', 'frustrant', 'énervant', 'irritant',
                'détester', 'haïr', 'mépriser', 'abhorrer', 'ne pas aimer',
                'triste', 'en colère', 'contrarié', 'déçu', 'frustré',
                'cassé', 'défectueux', 'endommagé', 'sans valeur', 'inférieur', 'négatif'
            },
            'intensifiers': {
                'très', 'vraiment', 'extrêmement', 'incroyablement', 'absolument',
                'totalement', 'complètement', 'assez', 'plutôt', 'vraiment'
            },
            'negations': {
                'ne', 'pas', 'non', 'jamais', 'rien', 'personne', 'aucun', 'ni'
            }
        }
        
        # German lexicon
        self.lexicons['de'] = {
            'positive': {
                'ausgezeichnet', 'unglaublich', 'wunderbar', 'fantastisch', 'toll', 'großartig',
                'brilliant', 'außergewöhnlich', 'herrlich', 'perfekt', 'gut', 'schön',
                'lieben', 'mögen', 'genießen', 'schätzen', 'verehren', 'bewundern',
                'glücklich', 'freude', 'zufrieden', 'erfreut', 'begeistert', 'entzückt',
                'nützlich', 'wertvoll', 'vorteilhaft', 'positiv', 'angenehm', 'herrlich'
            },
            'negative': {
                'schrecklich', 'furchtbar', 'entsetzlich', 'ekelhaft', 'erbärmlich', 'nutzlos',
                'schlecht', 'schlimm', 'enttäuschend', 'frustrierend', 'ärgerlich', 'störend',
                'hassen', 'verabscheuen', 'verachten', 'nicht mögen', 'ablehnen',
                'traurig', 'wütend', 'verärgert', 'enttäuscht', 'frustriert',
                'kaputt', 'defekt', 'beschädigt', 'wertlos', 'minderwertig', 'negativ'
            },
            'intensifiers': {
                'sehr', 'wirklich', 'extrem', 'unglaublich', 'absolut', 'völlig',
                'komplett', 'ziemlich', 'recht', 'durchaus', 'wahrhaft'
            },
            'negations': {
                'nicht', 'nein', 'nie', 'niemals', 'nichts', 'niemand', 'kein', 'keine'
            }
        }
        
        # Simplified lexicons for other languages (in practice, these would be more comprehensive)
        self.lexicons['ja'] = {
            'positive': {
                '素晴らしい', '最高', '良い', '好き', '愛', '嬉しい', '幸せ', '満足',
                'すごい', '素敵', '美しい', '完璧', '優秀', '楽しい', '快適', '便利'
            },
            'negative': {
                'ひどい', '最悪', '悪い', '嫌い', '憎む', '悲しい', '怒り', '不満',
                '失望', '困る', '面倒', 'だめ', '無駄', '壊れた', '欠陥', '問題'
            },
            'intensifiers': {'とても', 'すごく', 'かなり', '非常に', '本当に', '実に'},
            'negations': {'ない', 'ではない', '決して', '全然', '少しも', 'まったく'}
        }
        
        self.lexicons['zh'] = {
            'positive': {
                '优秀', '很好', '棒', '爱', '喜欢', '高兴', '快乐', '满意',
                '完美', '美丽', '精彩', '出色', '卓越', '杰出', '令人惊叹', 'fantastic'
            },
            'negative': {
                '糟糕', '坏', '讨厌', '恨', '愤怒', '悲伤', '失望', '不满',
                '可怕', '恶心', '无用', '破碎', '有缺陷的', '问题', '错误'
            },
            'intensifiers': {'非常', '很', '极其', '特别', '相当', '十分', '真的'},
            'negations': {'不', '没', '没有', '绝不', '从不', '决不', '毫不'}
        }
        
        # Add basic lexicons for other supported languages
        for lang_code in self.language_configs:
            if lang_code not in self.lexicons:
                # Create minimal lexicon (in practice, would load from external resources)
                self.lexicons[lang_code] = {
                    'positive': {'good', 'great', 'excellent', 'love', 'like', 'happy'},
                    'negative': {'bad', 'terrible', 'hate', 'dislike', 'sad', 'angry'},
                    'intensifiers': {'very', 'really', 'extremely'},
                    'negations': {'not', 'no', 'never'}
                }
        
        logger.info(f"Loaded lexicons for {len(self.lexicons)} languages")
    
    def get_lexicon(self, language: str) -> Dict[str, Set[str]]:
        """Get lexicon for specified language"""
        return self.lexicons.get(language, self.lexicons[self.default_language])
    
    def is_language_supported(self, language: str) -> bool:
        """Check if language is supported"""
        return language in self.language_configs
    
    def get_supported_languages(self) -> List[str]:
        """Get list of supported language codes"""
        return list(self.language_configs.keys())
    
    def get_language_info(self, language: str) -> Optional[LanguageConfig]:
        """Get language configuration"""
        return self.language_configs.get(language)
    
    def detect_language(self, text: str) -> str:
        """Simple language detection based on character patterns"""
        # This is a very basic implementation
        # In production, you'd use proper language detection libraries
        
        # Check for non-Latin scripts
        if any(ord(char) >= 0x4E00 and ord(char) <= 0x9FFF for char in text):
            return 'zh'  # Chinese characters
        
        if any(ord(char) >= 0x3040 and ord(char) <= 0x309F for char in text) or \
           any(ord(char) >= 0x30A0 and ord(char) <= 0x30FF for char in text):
            return 'ja'  # Japanese characters
        
        if any(ord(char) >= 0x0600 and ord(char) <= 0x06FF for char in text):
            return 'ar'  # Arabic characters
        
        if any(ord(char) >= 0x0400 and ord(char) <= 0x04FF for char in text):
            return 'ru'  # Cyrillic characters
        
        # Simple word-based detection for Latin scripts
        words = text.lower().split()
        
        # Spanish indicators
        spanish_words = {'el', 'la', 'de', 'que', 'y', 'en', 'un', 'es', 'se', 'no', 'te', 'lo'}
        if sum(1 for word in words if word in spanish_words) / len(words) > 0.1 if words else False:
            return 'es'
        
        # French indicators
        french_words = {'le', 'de', 'et', 'à', 'un', 'il', 'être', 'et', 'en', 'avoir', 'que', 'pour'}
        if sum(1 for word in words if word in french_words) / len(words) > 0.1 if words else False:
            return 'fr'
        
        # German indicators
        german_words = {'der', 'die', 'und', 'in', 'den', 'von', 'zu', 'das', 'mit', 'sich', 'des', 'auf'}
        if sum(1 for word in words if word in german_words) / len(words) > 0.1 if words else False:
            return 'de'
        
        # Default to English
        return 'en'
    
    def get_confidence_modifier(self, language: str) -> float:
        """Get confidence modifier for language"""
        config = self.language_configs.get(language)
        return config.confidence_modifier if config else 1.0
    
    def preprocess_text_for_language(self, text: str, language: str) -> str:
        """Language-specific text preprocessing"""
        config = self.language_configs.get(language)
        
        if not config:
            return text
        
        # Handle right-to-left languages
        if config.rtl:
            # Basic RTL handling (in practice, would be more sophisticated)
            # For now, just normalize whitespace
            text = ' '.join(text.split())
        
        # Language-specific normalization
        if language == 'de':
            # German: normalize umlauts if needed
            # This is simplified - real implementation would handle more cases
            pass
        elif language == 'zh':
            # Chinese: handle traditional/simplified conversion if needed
            pass
        elif language == 'ar':
            # Arabic: handle diacritics and normalization
            pass
        
        return text
    
    def add_custom_lexicon(self, language: str, lexicon: Dict[str, Set[str]]):
        """Add or update custom lexicon for a language"""
        if language not in self.language_configs:
            # Add basic language config
            self.language_configs[language] = LanguageConfig(
                language, f"Custom_{language}", False, ['lexicon'], 0.8
            )
        
        self.lexicons[language] = lexicon
        logger.info(f"Added custom lexicon for language: {language}")
    
    def export_lexicon(self, language: str, format: str = 'json') -> str:
        """Export lexicon in specified format"""
        if language not in self.lexicons:
            raise ValueError(f"Language {language} not supported")
        
        lexicon = self.lexicons[language]
        
        if format.lower() == 'json':
            # Convert sets to lists for JSON serialization
            json_lexicon = {
                key: list(value) if isinstance(value, set) else value
                for key, value in lexicon.items()
            }
            return json.dumps(json_lexicon, indent=2, ensure_ascii=False)
        elif format.lower() == 'txt':
            # Simple text format
            output = []
            for category, words in lexicon.items():
                output.append(f"# {category.upper()}")
                if isinstance(words, set):
                    output.extend(sorted(words))
                output.append("")  # Empty line
            return "\\n".join(output)
        else:
            raise ValueError(f"Unsupported format: {format}")
    
    def load_lexicon_from_file(self, language: str, file_path: str):
        """Load lexicon from external file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                if file_path.endswith('.json'):
                    data = json.load(f)
                    # Convert lists back to sets
                    lexicon = {
                        key: set(value) if isinstance(value, list) else value
                        for key, value in data.items()
                    }
                    self.lexicons[language] = lexicon
                else:
                    # Simple text format parsing
                    # Implementation would depend on specific format
                    pass
            
            logger.info(f"Loaded lexicon for {language} from {file_path}")
            
        except Exception as e:
            logger.error(f"Error loading lexicon from {file_path}: {e}")
            raise