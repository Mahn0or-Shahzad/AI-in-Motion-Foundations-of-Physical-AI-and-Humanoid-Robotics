import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

export default function TranslateButton() {
  const [isTranslated, setIsTranslated] = useState(false);

  useEffect(() => {
    // Check initial translation state from window variable
    if (typeof window !== 'undefined') {
      setIsTranslated(!!window.isUrduMode);
    }
  }, []);

  const handleTranslate = async () => {
    if (window.translateToUrdu) {
      await window.translateToUrdu();
      setIsTranslated(prev => !prev);
    }
  };

  return (
    <button
      className={`${styles.translateButton} ${isTranslated ? styles.translated : ''}`}
      onClick={handleTranslate}
      aria-label={isTranslated ? "Switch to English" : "Translate to Urdu"}
    >
      {isTranslated ? 'English' : 'Translate to Urdu'}
    </button>
  );
}