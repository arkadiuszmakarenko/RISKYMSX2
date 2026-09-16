/**
 * CH32V407/V467 Reference Manual — Site Application
 * Handles sidebar navigation, dynamic chapter loading, and full-text search.
 */

(function() {
'use strict';

const CHAPTERS = typeof TOC_DATA !== 'undefined' ? TOC_DATA : [];
const CONTENT_DIR = '';

// ========== State ==========
let currentChapter = null;
let searchIndex = null;
let searchTimeout = null;

// ========== DOM ==========
const sidebar = document.getElementById('sidebar');
const content = document.getElementById('content');
const loading = document.getElementById('loading');
const searchInput = document.getElementById('search-input');
const searchResults = document.getElementById('search-results');
const sidebarToggle = document.getElementById('sidebar-toggle');

// ========== Initialize ==========
function init() {
    buildSidebar();
    loadSearchIndex();

    // Handle hash navigation
    window.addEventListener('hashchange', handleHashChange);

    // Search
    searchInput.addEventListener('input', handleSearchInput);
    searchInput.addEventListener('focus', handleSearchFocus);
    document.addEventListener('click', (e) => {
        if (!e.target.closest('#search-box')) {
            searchResults.classList.remove('visible');
        }
    });

    // Sidebar toggle (mobile)
    if (sidebarToggle) {
        sidebarToggle.addEventListener('click', () => sidebar.classList.toggle('open'));
    }

    // Close sidebar on nav click (mobile)
    sidebar.addEventListener('click', (e) => {
        if (e.target.classList.contains('sidebar-link') || e.target.classList.contains('sidebar-sublink')) {
            if (window.innerWidth <= 900) {
                sidebar.classList.remove('open');
            }
        }
    });

    // Initial load
    handleHashChange();
}

// ========== Sidebar ==========
function buildSidebar() {
    let html = '';

    // Front matter link
    html += '<div class="sidebar-section">';
    html += '<div class="sidebar-section-title">Front Matter</div>';
    html += `<a class="sidebar-link" href="#front_matter" data-file="front_matter.html">Introduction & Register Attributes</a>`;
    html += '</div>';

    // Chapters
    html += '<div class="sidebar-section">';
    html += '<div class="sidebar-section-title">Chapters</div>';

    CHAPTERS.forEach(ch => {
        const title = ch.title.replace(/^Chapter \d+:\s*/, '');
        html += `<a class="sidebar-link" href="#ch${ch.num}" data-file="${ch.file}">`;
        html += `<strong>Ch ${ch.num}:</strong> ${escapeHtml(title)}</a>`;

        // Sub-sections (h2 only for sidebar)
        ch.sections.forEach(sec => {
            if (sec.tag === 'h2') {
                html += `<a class="sidebar-sublink" href="#ch${ch.num}/${sec.id}">${escapeHtml(sec.text)}</a>`;
            }
        });
    });

    html += '</div>';

    sidebar.innerHTML = html;
}

function updateSidebarActive(chapterKey) {
    // Remove all active states
    sidebar.querySelectorAll('.sidebar-link').forEach(el => el.classList.remove('active'));

    // Set active on current chapter link
    const activeLink = sidebar.querySelector(`[data-file]`);
    sidebar.querySelectorAll('.sidebar-link').forEach(el => {
        const href = el.getAttribute('href');
        if (href === '#' + chapterKey || href.startsWith('#' + chapterKey + '/')) {
            el.classList.add('active');
            // Scroll into view in sidebar
            el.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
        }
    });
}

// ========== Content Loading ==========
function handleHashChange() {
    const hash = window.location.hash.substring(1); // Remove #

    if (!hash) {
        // Default to front matter
        loadContent('front_matter', 'front_matter.html');
        return;
    }

    // Parse hash: "ch5" or "ch5/section-slug" or "front_matter"
    let file, key, anchor;

    if (hash === 'front_matter') {
        file = 'front_matter.html';
        key = 'front_matter';
        anchor = null;
    } else {
        const parts = hash.split('/');
        key = parts[0]; // e.g. "ch5"
        anchor = parts.length > 1 ? parts[1] : null;

        const chNum = parseInt(key.replace('ch', ''));
        if (isNaN(chNum) || chNum < 1 || chNum > 35) {
            loadContent('front_matter', 'front_matter.html');
            return;
        }
        file = `ch${String(chNum).padStart(2, '0')}.html`;
    }

    loadContent(key, file, anchor);
}

function loadContent(key, file, anchor) {
    showLoading();

    fetch(CONTENT_DIR + file)
        .then(resp => {
            if (!resp.ok) throw new Error('Failed to load ' + file);
            return resp.text();
        })
        .then(html => {
            // Inject content
            content.innerHTML = html;

            // Add chapter navigation (prev/next)
            addChapterNav(key);

            // Update active sidebar link
            updateSidebarActive(key);

            // Scroll to anchor if specified
            if (anchor) {
                // Wait a bit for content to render
                setTimeout(() => {
                    const el = document.getElementById(anchor);
                    if (el) {
                        el.scrollIntoView({ block: 'start', behavior: 'smooth' });
                    }
                }, 100);
            } else {
                // Scroll to top
                window.scrollTo(0, 0);
            }

            currentChapter = key;
            hideLoading();
        })
        .catch(err => {
            content.innerHTML = `<h1>Error</h1><p>Failed to load content: ${escapeHtml(err.message)}</p>`;
            hideLoading();
        });
}

function addChapterNav(key) {
    if (key === 'front_matter') {
        // Only "next" link
        const nav = document.createElement('div');
        nav.className = 'chapter-nav';
        nav.innerHTML = `<a href="#ch1">Next: Chapter 1 →</a>`;
        content.appendChild(nav);
        return;
    }

    const chNum = parseInt(key.replace('ch', ''));
    if (isNaN(chNum)) return;

    const prevKey = chNum === 1 ? 'front_matter' : `ch${chNum - 1}`;
    const prevFile = chNum === 1 ? 'front_matter.html' : `ch${String(chNum - 1).padStart(2, '0')}.html`;
    const prevTitle = chNum === 1 ? 'Front Matter' : `Chapter ${chNum - 1}`;

    const nextKey = chNum < 35 ? `ch${chNum + 1}` : null;
    const nextTitle = chNum < 35 ? `Chapter ${chNum + 1}` : '';

    const nav = document.createElement('div');
    nav.className = 'chapter-nav';

    let html = `<a href="#${prevKey}">← Previous: ${escapeHtml(prevTitle)}</a>`;
    if (nextKey) {
        html += `<a href="#${nextKey}">Next: ${escapeHtml(nextTitle)} →</a>`;
    } else {
        html += `<a class="disabled">End of Manual</a>`;
    }
    nav.innerHTML = html;
    content.appendChild(nav);
}

// ========== Search ==========
function loadSearchIndex() {
    fetch(CONTENT_DIR + 'search_index.json')
        .then(resp => resp.json())
        .then(data => { searchIndex = data; })
        .catch(() => { console.warn('Search index failed to load'); });
}

function handleSearchInput() {
    clearTimeout(searchTimeout);
    const query = searchInput.value.trim();

    if (query.length < 2) {
        searchResults.classList.remove('visible');
        return;
    }

    searchTimeout = setTimeout(() => performSearch(query), 200);
}

function handleSearchFocus() {
    if (searchInput.value.trim().length >= 2) {
        searchResults.classList.add('visible');
    }
}

function performSearch(query) {
    if (!searchIndex) return;

    const lowerQuery = query.toLowerCase();
    const results = [];
    const maxResults = 30;

    for (const entry of searchIndex) {
        if (results.length >= maxResults) break;

        const titleLower = entry.title.toLowerCase();
        let score = 0;
        let context = '';

        // Title match (highest score)
        if (titleLower.includes(lowerQuery)) {
            score += 100;
            context = entry.title;
        }

        // Heading matches
        for (const h of entry.headings) {
            if (h.toLowerCase().includes(lowerQuery)) {
                score += 50;
                if (!context) context = h;
                break;
            }
        }

        // Caption matches (figures/tables)
        for (const c of entry.captions) {
            if (c.toLowerCase().includes(lowerQuery)) {
                score += 30;
                if (!context) context = c;
                break;
            }
        }

        // Full text match
        const ftLower = entry.fulltext.toLowerCase();
        const ftIdx = ftLower.indexOf(lowerQuery);
        if (ftIdx >= 0) {
            score += 10;
            if (!context) {
                // Extract context around the match
                const start = Math.max(0, ftIdx - 40);
                const end = Math.min(entry.fulltext.length, ftIdx + query.length + 40);
                context = '...' + entry.fulltext.substring(start, end) + '...';
            }
        }

        if (score > 0) {
            results.push({
                chapter: entry.chapter,
                title: entry.title,
                context: context,
                score: score
            });
        }
    }

    // Sort by score
    results.sort((a, b) => b.score - a.score);

    renderSearchResults(results, query);
}

function renderSearchResults(results, query) {
    if (results.length === 0) {
        searchResults.innerHTML = '<div class="search-result-item"><div class="sr-title">No results found</div></div>';
    } else {
        searchResults.innerHTML = results.map(r => {
            const chNum = r.chapter;
            const href = `#ch${chNum}`;
            return `<a class="search-result-item" href="${href}">
                <div class="sr-title">${escapeHtml(r.title)}</div>
                <div class="sr-context">${highlightQuery(r.context, query)}</div>
            </a>`;
        }).join('');
    }
    searchResults.classList.add('visible');
}

function highlightQuery(text, query) {
    const escaped = escapeHtml(text);
    const regex = new RegExp('(' + escapeRegex(query) + ')', 'gi');
    return escaped.replace(regex, '<mark>$1</mark>');
}

// ========== Utilities ==========
function showLoading() { loading.classList.remove('hidden'); }
function hideLoading() { loading.classList.add('hidden'); }

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function escapeRegex(text) {
    return text.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

function slugify(text) {
    return text
        .replace(/[^\w.\-]/g, '-')
        .replace(/-+/g, '-')
        .replace(/^-+|-+$/g, '')
        .toLowerCase();
}

// ========== Boot ==========
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}

})();