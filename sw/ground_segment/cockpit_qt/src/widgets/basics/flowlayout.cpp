#include <utility>
#include <QtWidgets>
#include "flowlayout.h"
//! [1]
FlowLayout::FlowLayout(QWidget *parent, int margin, int hSpacing, int vSpacing)
    : QLayout(parent), m_hSpace(hSpacing), m_vSpace(vSpacing)
{
    setContentsMargins(margin, margin, margin, margin);
}

FlowLayout::FlowLayout(int margin, int hSpacing, int vSpacing)
    : m_hSpace(hSpacing), m_vSpace(vSpacing)
{
    setContentsMargins(margin, margin, margin, margin);
}


FlowLayout::~FlowLayout()
{
    QLayoutItem *item;
    while ((item=takeAt(0)) != nullptr) {
        delete item;
    }
}

void FlowLayout::addItem(QLayoutItem *item)
{
    itemList.append(item);
}

int FlowLayout::horizontalSpacing() const
{
    if (m_hSpace >= 0) {
        return m_hSpace;
    } else {
        return smartSpacing(QStyle::PM_LayoutHorizontalSpacing);
    }
}

int FlowLayout::verticalSpacing() const
{
    if (m_vSpace >= 0) {
        return m_vSpace;
    } else {
        return smartSpacing(QStyle::PM_LayoutVerticalSpacing);
    }
}


int FlowLayout::count() const
{
    return itemList.size();
}

QLayoutItem *FlowLayout::itemAt(int index) const
{
    return itemList.value(index);
}

QLayoutItem *FlowLayout::takeAt(int index)
{
    if (index >= 0 && index < itemList.size())
        return itemList.takeAt(index);
    else
        return nullptr;
}


Qt::Orientations FlowLayout::expandingDirections() const
{
    return Qt::Horizontal;
}


bool FlowLayout::hasHeightForWidth() const
{
    return true;
}


QSize FlowLayout::sizeHint() const
{
    int w = contentsRect().width();
    return QSize(w, heightForWidth(w));
    //return minimumSize();
}


int FlowLayout::heightForWidth(int width) const {
    const int height = doLayout(QRect(0, 0, width, 0), true, nullptr); // jpo38: set added parameter to NULL here
    return height;
}

void FlowLayout::setGeometry(const QRect &rect) {
    QLayout::setGeometry(rect);

    // jpo38: update minSize from here, force layout to consider it if it changed
    QSize oldSize = minSize;
    doLayout(rect, false,&minSize);
    if ( oldSize != minSize )
    {
        // force layout to consider new minimum size!
        invalidate();
    }
}

QSize FlowLayout::minimumSize() const {
    // jpo38: Simply return computed min size

    //return minSize;
    QSize size;
    for (const QLayoutItem *item : std::as_const(itemList))
        size = size.expandedTo(item->minimumSize());

    const QMargins margins = contentsMargins();
    size += QSize(margins.left() + margins.right(), margins.top() + margins.bottom());

    return QSize(size.width(), minSize.height());
}

int FlowLayout::doLayout(const QRect &rect, bool testOnly,QSize* pMinSize) const {
    int left, top, right, bottom;
    getContentsMargins(&left, &top, &right, &bottom);
    QRect effectiveRect = rect.adjusted(+left, +top, -right, -bottom);
    int x = effectiveRect.x();
    int y = effectiveRect.y();
    int lineHeight = 0;

    // jpo38: store max X
    int maxX = 0;

    for (auto&& item : itemList) {
        QWidget *wid = item->widget();
        int spaceX = horizontalSpacing();
        if (spaceX == -1)
            spaceX = wid->style()->layoutSpacing(QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Horizontal);
        int spaceY = verticalSpacing();
        if (spaceY == -1)
            spaceY = wid->style()->layoutSpacing(QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Vertical);
        int nextX = x + item->sizeHint().width() + spaceX;
        if (nextX - spaceX > effectiveRect.right() && lineHeight > 0) {
            x = effectiveRect.x();
            y = y + lineHeight + spaceY;
            nextX = x + item->sizeHint().width() + spaceX;
            lineHeight = 0;
        }

        if (!testOnly)
            item->setGeometry(QRect(QPoint(x, y), item->sizeHint()));

        // jpo38: update max X based on current position
        maxX = qMax( maxX, x + item->sizeHint().width() - rect.x() + left );

        x = nextX;
        lineHeight = qMax(lineHeight, item->sizeHint().height());
    }

    // jpo38: save height/width as max height/xidth in pMinSize is specified
    int height = y + lineHeight - rect.y() + bottom;
    if ( pMinSize )
    {
        pMinSize->setHeight( height );
        pMinSize->setWidth( maxX );
    }
    return height;
}


int FlowLayout::smartSpacing(QStyle::PixelMetric pm) const
{
    QObject *parent = this->parent();
    if (!parent) {
        return -1;
    } else if (parent->isWidgetType()) {
        QWidget *pw = static_cast<QWidget *>(parent);
        return pw->style()->pixelMetric(pm, nullptr, pw);
    } else {
        return static_cast<QLayout *>(parent)->spacing();
    }
}
//! [12]
