// AI-powered Urdu translation utility
export async function translateToUrdu() {
  // Get all text elements on the page
  const textElements = Array.from(
    document.querySelectorAll('h1, h2, h3, h4, h5, h6, p, span, div, li, td, th, a, .hero__title, .hero__subtitle, .navbar__title')
  ).filter(el => el.textContent.trim().length > 0 && !el.classList.contains('translate-ignore'));

  // Store original English content for toggling
  if (!window.originalContent) {
    window.originalContent = {};
  }

  // Toggle between English and Urdu
  if (window.isUrduMode) {
    // Restore original English content
    textElements.forEach(el => {
      const elementKey = el.id || el.textContent.substring(0, 50);
      if (window.originalContent[elementKey]) {
        el.textContent = window.originalContent[elementKey];
      }
    });
    window.isUrduMode = false;
  } else {
    // Save original content and translate to Urdu
    for (const el of textElements) {
      const originalText = el.textContent;
      const elementKey = el.id || originalText.substring(0, 50);

      if (!window.originalContent[elementKey]) {
        window.originalContent[elementKey] = originalText;
      }

      try {
        // Simulate API call to AI translation service
        const urduText = await simulateUrduTranslation(originalText);
        el.textContent = urduText;
      } catch (error) {
        console.warn('Translation failed for:', originalText, error);
      }
    }
    window.isUrduMode = true;
  }
}

// Simulate AI translation API call
async function simulateUrduTranslation(text) {
  // This would typically be an API call to an AI translation service
  // For demonstration, we'll return placeholder translations
  const translationMap = {
    'AI in Motion': 'مصنوعی ذہانت حرکت میں',
    'Foundations of Physical AI and Humanoid Robotics': 'فزیکل ای آئی اور ہیومنوائڈ روبوٹکس کی بنیادیں',
    'Start Learning': 'سیکھنا شروع کریں',
    'Open Book': 'کتاب کھولیں',
    'Physical AI Basics': 'فزیکل ای آئی کی بنیادیں',
    'Introduction to embodied intelligence fundamentals.': 'جسمانی شدہ ذہانت کے بنیادیات کا تعارف۔',
    'Humanoid Robotics': 'ہیومنوائڈ روبوٹکس',
    'Learn structure and control of humanoid robots.': 'ہیومنوائڈ روبوٹس کی ساخت اور کنٹرول سیکھیں۔',
    'Motion Intelligence': 'حرکت کی ذہانت',
    'Explore movement, balance, and real-world physics.': 'حرکت، توازن اور حقیقی دنیا کی فزکس کا جائزہ لیں۔',
    'Tutorial': 'ٹیوٹوریل',
    'Blog': ' بلاگ',
    'GitHub': 'گیت ہب',
    'Docs': 'دستاویزات',
    'Community': 'برادری',
    'More': 'مزید',
    'Stack Overflow': 'اسٹیک اوور فلو',
    'Discord': 'ڈسکورڈ',
    'Twitter': 'ٹویٹر',
  };

  return translationMap[text] || `[URDU: ${text}]`;
}

// Expose function globally for navbar component access
if (typeof window !== 'undefined') {
  window.translateToUrdu = translateToUrdu;
}